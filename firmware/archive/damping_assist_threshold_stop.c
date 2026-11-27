/**
 ******************************************************************************
 * @file    damping_assist_manual_buttons_threshold_clean.c
 * @brief   Manual-button-triggered continuous hip damping control for XM10 + H10
 *
 * IMPORTANT
 * ----------------------------------------------------------------------------
 * This is a PURE C source file. It contains no Python patch/generator code.
 *
 * Manual perturbation trigger:
 *   XM BTN_1 -> ES slip
 *   XM BTN_2 -> LS slip
 *   XM BTN_3 -> early-swing trip
 *
 * Event side is selected with:
 *   g_manual_event_side = 1  -> RIGHT
 *   g_manual_event_side = 2  -> LEFT
 *
 * Continuous damping law while ACTIVE:
 *
 *      tau(t) = - B_eff * qdot(t)
 *
 * qdot is calculated from H10 hip encoder angle and low-pass filtered.
 *
 * STOP RULE -- NO ASSISTANCE DURATION TIMER
 * ----------------------------------------------------------------------------
 * There is NO min-active duration and NO max-active duration in this version.
 * The reaction controller stops ONLY from threshold-confirmed velocity reversal.
 *
 * ES slip / early-swing trip:
 *   1) primary flexion-positive hip velocity must first satisfy
 *          qdot_flex >= +g_reversal_threshold_dps
 *   2) after that, controller stops only when
 *          qdot_flex <= -g_reversal_threshold_dps
 *
 * LS slip:
 *   1) primary flexion-positive hip velocity must first satisfy
 *          qdot_flex <= -g_reversal_threshold_dps
 *   2) after that, controller stops only when
 *          qdot_flex >= +g_reversal_threshold_dps
 *
 * Therefore small zero-near sign changes such as +1 -> -1 deg/s do NOT stop
 * the controller when the threshold is, for example, 20 deg/s.
 *
 * TRIP TIMING
 * ----------------------------------------------------------------------------
 * BTN_3 means obstacle contact. Trip damping does not start immediately.
 * It waits until the perturbed foot is observed airborne and then contacts
 * the ground again through H10 foot-contact status.
 *
 * If you want to manually force the trip foot-contact transition for a bench
 * test, set:
 *      g_test_trip_foot_contact = 1
 *
 * No fixed-delay fallback is used in this version.
 *
 * USB STREAM
 * ----------------------------------------------------------------------------
 * Kept at the original 12-float payload to minimize changes while debugging:
 *   [0] RH angle
 *   [1] LH angle
 *   [2] RH qdot
 *   [3] LH qdot
 *   [4] RH B_eff
 *   [5] LH B_eff
 *   [6] RH tau raw
 *   [7] LH tau raw
 *   [8] event
 *   [9] state
 *   [10] angle scale
 *   [11] mean COM velocity
 *
 * Actual commanded torque is still available in Live Expressions:
 *      g_right_tau_cmd_nm
 *      g_left_tau_cmd_nm
 *
 * SAFETY
 * ----------------------------------------------------------------------------
 * g_control_ON defaults to 0, so calculated raw torque can be observed without
 * sending nonzero torque to H10.
 *
 * Software torque clamp is present but defaults OFF.
 ******************************************************************************
 */

#include "xm_api.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

/* ========================================================================== */
/* 1. EVENT / STATE TYPES                                                     */
/* ========================================================================== */

typedef enum {
    PERT_NONE             = 0,
    PERT_ES_SLIP          = 1,
    PERT_LS_SLIP          = 2,
    PERT_EARLY_SWING_TRIP = 3
} PerturbationType_t;

typedef enum {
    LEG_NONE  = 0,
    LEG_RIGHT = 1,
    LEG_LEFT  = 2
} LegSide_t;

typedef enum {
    CTRL_IDLE = 0,
    CTRL_WAIT_TRIP_FOOT_CONTACT = 1,
    CTRL_ACTIVE = 2
} ReactionState_t;

/* ========================================================================== */
/* 2. TUNING PARAMETERS                                                       */
/* ========================================================================== */

/* Master output arm:
 * 0 = calculate raw torque but send 0 Nm
 * 1 = send calculated torque
 */
volatile uint16_t g_control_ON = 0U;

/* Optional software clamp. OFF by default. */
volatile uint16_t g_enable_torque_clamp = 0U;
volatile float    g_torque_limit_nm     = 10.0f;

/*
 * Sign conventions.
 *
 * flexion_sign:
 *   +1 : positive encoder qdot = hip flexion
 *   -1 : negative encoder qdot = hip flexion
 *
 * torque_sign:
 *   +1 : XM positive torque acts in positive encoder-angle direction
 *   -1 : XM positive torque acts in negative encoder-angle direction
 */
volatile int16_t g_right_flexion_sign = +1;
volatile int16_t g_left_flexion_sign  = +1;
volatile int16_t g_right_torque_sign  = +1;
volatile int16_t g_left_torque_sign   = +1;

/* Damping coefficients [Nm*s/rad].
 * Current test default = 50.0, as requested.
 * Change these in Live Expressions if needed.
 */
volatile float g_k_es_impact   = 50.0f;
volatile float g_k_es_stance   = 50.0f;
volatile float g_k_ls_slip     = 50.0f;
volatile float g_k_ls_stance   = 50.0f;
volatile float g_k_trip_impact = 50.0f;
volatile float g_k_trip_stance = 50.0f;

/* Slip stance-leg scaling:
 * angle_scale = 1 + gain * |qR - qL|
 */
volatile float g_angle_gain_per_deg = 0.02f;

/* Trip COM scaling is available but disabled until COM velocity is validated. */
volatile uint16_t g_trip_com_scaling_enable = 0U;
volatile float    g_trip_com_gain_per_mps   = 1.0f;

/*
 * ONLY controller-stop parameter.
 *
 * ES / Trip : must observe +TH first, then -TH -> STOP
 * LS        : must observe -TH first, then +TH -> STOP
 */
volatile float g_reversal_threshold_dps = 20.0f;

/* Hip velocity estimator LPF cutoff. */
volatile float g_vel_lpf_hz = 15.0f;

/* ========================================================================== */
/* 3. MANUAL TEST INPUTS                                                      */
/* ========================================================================== */

/* Physical buttons choose event type.
 * This variable chooses perturbation side:
 * 1 = right, 2 = left.
 */
volatile uint16_t g_manual_event_side = LEG_RIGHT;

/* For trip, use H10 foot-contact transition automatically. */
volatile uint16_t g_manual_trip_use_h10_foot_contact = 1U;

/* Optional Live-Expression manual trigger, independent of physical buttons. */
volatile uint16_t g_test_trigger    = 0U;
volatile uint16_t g_test_event_type = PERT_ES_SLIP;
volatile uint16_t g_test_event_side = LEG_RIGHT;

/* For trip bench testing, set this to 1 to force "perturbed foot contact". */
volatile uint16_t g_test_trip_foot_contact = 0U;

/* Optional externally estimated COM velocity input for trip scaling. */
volatile float g_test_com_velocity_mps = 0.0f;

/* ========================================================================== */
/* 4. PUBLIC DIAGNOSTICS / LIVE EXPRESSIONS                                   */
/* ========================================================================== */

/* Button diagnostics */
volatile uint16_t g_manual_btn1_state_dbg = XM_RELEASED;
volatile uint16_t g_manual_btn2_state_dbg = XM_RELEASED;
volatile uint16_t g_manual_btn3_state_dbg = XM_RELEASED;
volatile uint16_t g_manual_last_button    = 0U;
volatile uint32_t g_manual_trigger_count  = 0U;

/* Trip diagnostics */
volatile uint16_t g_manual_trip_waiting_for_contact = 0U;
volatile uint16_t g_manual_trip_seen_airborne       = 0U;

/* Kinematics */
volatile float g_right_hip_angle_deg = 0.0f;
volatile float g_left_hip_angle_deg  = 0.0f;
volatile float g_right_hip_vel_dps   = 0.0f;
volatile float g_left_hip_vel_dps    = 0.0f;

/* Flexion-positive qdot */
volatile float g_right_flex_vel_dps = 0.0f;
volatile float g_left_flex_vel_dps  = 0.0f;

/* Current effective damping coefficients */
volatile float g_right_B_eff = 0.0f;
volatile float g_left_B_eff  = 0.0f;

/* Scaling */
volatile float g_interleg_angle_deg = 0.0f;
volatile float g_angle_scale        = 1.0f;
volatile float g_trip_mean_com_velocity_mps = 0.0f;
volatile float g_trip_com_scale             = 1.0f;

/* Torque diagnostics */
volatile float g_right_tau_raw_nm = 0.0f;
volatile float g_left_tau_raw_nm  = 0.0f;
volatile float g_right_tau_cmd_nm = 0.0f;
volatile float g_left_tau_cmd_nm  = 0.0f;

/* Controller state */
volatile uint16_t g_active_event_type = PERT_NONE;
volatile uint16_t g_active_event_side = LEG_NONE;
volatile uint16_t g_reaction_state_dbg = CTRL_IDLE;

/*
 * Threshold diagnostics:
 *
 * 0 = harmful-direction threshold has not yet been confirmed
 * 1 = first threshold confirmed; now waiting for opposite threshold
 */
volatile uint16_t g_threshold_first_crossed_dbg = 0U;

/*
 * Latched stop diagnostics.
 * Useful because Live Expressions can miss a short event.
 */
volatile uint32_t g_reaction_stop_count = 0U;
volatile float    g_last_stop_flex_vel_dps = 0.0f;

/* ========================================================================== */
/* 5. PRIVATE DEFINITIONS                                                     */
/* ========================================================================== */

#define DEG2RAD               0.01745329251994329577f
#define TWO_PI_F              6.28318530717958647692f
#define ANGLE_EPS_DEG         0.005f
#define VEL_STALE_TIMEOUT_MS  120U
#define VEL_MAX_GAP_MS        60U

#define USB_STREAM_ID         0xF0U
#define USB_DEBUG_PERIOD_MS   200U

typedef struct {
    bool     initialized;
    float    last_angle_deg;
    uint32_t last_change_tick;
    float    vel_dps;
} HipVelEstimator_t;

typedef struct {
    ReactionState_t     state;
    PerturbationType_t  type;
    LegSide_t           side;

    /* Trip COM accumulation between obstacle contact and foot contact */
    float    com_velocity_sum;
    uint32_t com_velocity_count;
    float    mean_com_velocity_mps;
    float    trip_com_scale;

    /*
     * +1 = harmful primary direction is flexion
     * -1 = harmful primary direction is extension
     */
    int8_t primary_harmful_direction;

    /*
     * Becomes true only after the FIRST threshold in the expected harmful
     * direction has been observed.
     */
    bool threshold_crossed_first;
} ReactionController_t;

/*
 * Keep original 12-float payload while debugging.
 */
typedef struct {
    float data[12];
} StreamData_t;

/* ========================================================================== */
/* 6. STATIC DATA                                                             */
/* ========================================================================== */

static XmTsmHandle_t s_tsm;

static HipVelEstimator_t s_rh_vel;
static HipVelEstimator_t s_lh_vel;
static ReactionController_t s_ctrl;
static StreamData_t s_stream;

static XmBtnState_t s_prevBtn1 = XM_RELEASED;
static XmBtnState_t s_prevBtn2 = XM_RELEASED;
static XmBtnState_t s_prevBtn3 = XM_RELEASED;

static bool s_manualTripWaiting      = false;
static bool s_manualTripSeenAirborne = false;

static uint32_t s_usb_debug_tick = 0U;

/* ========================================================================== */
/* 7. PROTOTYPES                                                              */
/* ========================================================================== */

static void Off_Loop(void);
static void Standby_Loop(void);
static void Active_Entry(void);
static void Active_Loop(void);
static void Active_Exit(void);

static void  _ResetVelocityEstimator(HipVelEstimator_t *e);
static float _UpdateHipVelocity(HipVelEstimator_t *e, float angle_deg, uint32_t now);
static float _LpfAlpha(float cutoff_hz, float dt_s);

static void _ResetReactionController(void);
static void _StartReaction(PerturbationType_t type, LegSide_t side);
static void _RunReactionController(float *rh_tau, float *lh_tau);
static void _StopReaction(void);

static float _GetFlexVelDps(LegSide_t side);
static float _ComputeDampingTorqueNm(LegSide_t side, float B_eff, float qdot_dps);
static bool  _PrimaryVelocityReversed(void);

static bool  _ButtonRisingEdge(XmBtnState_t now, XmBtnState_t prev);
static bool  _GetH10FootContact(LegSide_t side);
static void  _UpdateManualPerturbationButtons(void);
static void  _UpdateManualTripFootContact(void);

static float _ClampFloat(float x, float lo, float hi);
static void  _UpdateUsbDebug(void);

/* ========================================================================== */
/* 8. PUBLIC DETECTOR / SENSOR INTERFACE                                      */
/* ========================================================================== */

void DampingAssist_Trigger(PerturbationType_t type, LegSide_t side)
{
    if ((type == PERT_NONE) || (side == LEG_NONE)) {
        return;
    }

    if (s_ctrl.state != CTRL_IDLE) {
        return;
    }

    _StartReaction(type, side);
}

void DampingAssist_UpdateComVelocity(float com_velocity_mps)
{
    if ((s_ctrl.type != PERT_EARLY_SWING_TRIP) ||
        (s_ctrl.state != CTRL_WAIT_TRIP_FOOT_CONTACT)) {
        return;
    }

    s_ctrl.com_velocity_sum += com_velocity_mps;
    s_ctrl.com_velocity_count++;
}

void DampingAssist_TripFootContact(LegSide_t side)
{
    if ((s_ctrl.type != PERT_EARLY_SWING_TRIP) ||
        (s_ctrl.state != CTRL_WAIT_TRIP_FOOT_CONTACT) ||
        (side != s_ctrl.side)) {
        return;
    }

    if (s_ctrl.com_velocity_count > 0U) {
        s_ctrl.mean_com_velocity_mps =
            s_ctrl.com_velocity_sum / (float)s_ctrl.com_velocity_count;
    } else {
        s_ctrl.mean_com_velocity_mps = 0.0f;
    }

    if (g_trip_com_scaling_enable == 1U) {
        s_ctrl.trip_com_scale =
            1.0f +
            g_trip_com_gain_per_mps *
            fabsf(s_ctrl.mean_com_velocity_mps);
    } else {
        s_ctrl.trip_com_scale = 1.0f;
    }

    g_trip_mean_com_velocity_mps = s_ctrl.mean_com_velocity_mps;
    g_trip_com_scale             = s_ctrl.trip_com_scale;

    /*
     * Trip reaction now begins.
     * No duration timer is started.
     */
    s_ctrl.state = CTRL_ACTIVE;
    s_ctrl.threshold_crossed_first = false;
    g_threshold_first_crossed_dbg = 0U;
}

/* ========================================================================== */
/* 9. XM ENTRY POINTS                                                         */
/* ========================================================================== */

void User_Setup(void)
{
    XM_SetExtPowerVoltage(XM_EXT_PWR_5V);

    s_tsm = XM_TSM_Create(XM_STATE_OFF);

    XmStateConfig_t off_conf = {
        .id = XM_STATE_OFF,
        .on_loop = Off_Loop
    };
    XM_TSM_AddState(s_tsm, &off_conf);

    XmStateConfig_t sb_conf = {
        .id = XM_STATE_STANDBY,
        .on_loop = Standby_Loop
    };
    XM_TSM_AddState(s_tsm, &sb_conf);

    XmStateConfig_t act_conf = {
        .id = XM_STATE_ACTIVE,
        .on_entry = Active_Entry,
        .on_loop  = Active_Loop,
        .on_exit  = Active_Exit
    };
    XM_TSM_AddState(s_tsm, &act_conf);

    /*
     * Original 12-channel metadata.
     * Do not expand payload while debugging Live Expressions.
     */
    XM_SetUsbCustomMeta(
        USB_STREAM_ID,
        "[{\"name\":\"RH_angle\",\"unit\":\"deg\"},"
        "{\"name\":\"LH_angle\",\"unit\":\"deg\"},"
        "{\"name\":\"RH_qdot\",\"unit\":\"dps\"},"
        "{\"name\":\"LH_qdot\",\"unit\":\"dps\"},"
        "{\"name\":\"RH_Beff\",\"unit\":\"Nms/rad\"},"
        "{\"name\":\"LH_Beff\",\"unit\":\"Nms/rad\"},"
        "{\"name\":\"RH_tau_raw\",\"unit\":\"Nm\"},"
        "{\"name\":\"LH_tau_raw\",\"unit\":\"Nm\"},"
        "{\"name\":\"event\",\"unit\":\"-\"},"
        "{\"name\":\"state\",\"unit\":\"-\"},"
        "{\"name\":\"angle_scale\",\"unit\":\"-\"},"
        "{\"name\":\"mean_COM_v\",\"unit\":\"m/s\"}]"
    );

    _ResetVelocityEstimator(&s_rh_vel);
    _ResetVelocityEstimator(&s_lh_vel);
    _ResetReactionController();

    XM_SetControlMode(XM_CTRL_MONITOR);
}

void User_Loop(void)
{
    if (!s_tsm) {
        return;
    }

    if (!XM_IsCmConnected()) {
        XM_TSM_TransitionTo(s_tsm, XM_STATE_OFF);
    }

    XM_TSM_Run(s_tsm);
    XM_IO_Update();
}

/* ========================================================================== */
/* 10. XM STATE MACHINE CALLBACKS                                             */
/* ========================================================================== */

static void Off_Loop(void)
{
    if (XM_IsCmConnected()) {
        XM_SetLedEffect(XM_LED_1, XM_LED_HEARTBEAT, 1000);
        XM_SendUsbDebugMessage("[DAMP] CM connected -> STANDBY\r\n");
        XM_TSM_TransitionTo(s_tsm, XM_STATE_STANDBY);
    }
}

static void Standby_Loop(void)
{
    if (XM.status.h10.h10Mode == XM_H10_MODE_ASSIST) {
        XM_SendUsbDebugMessage("[DAMP] H10 ASSIST -> ACTIVE\r\n");
        XM_TSM_TransitionTo(s_tsm, XM_STATE_ACTIVE);
    }
}

static void Active_Entry(void)
{
    XM_SetControlMode(XM_CTRL_TORQUE);

    _ResetVelocityEstimator(&s_rh_vel);
    _ResetVelocityEstimator(&s_lh_vel);
    _ResetReactionController();

    s_prevBtn1 = XM_GetButtonState(XM_BTN_1);
    s_prevBtn2 = XM_GetButtonState(XM_BTN_2);
    s_prevBtn3 = XM_GetButtonState(XM_BTN_3);

    s_manualTripWaiting = false;
    s_manualTripSeenAirborne = false;
    g_manual_trip_waiting_for_contact = 0U;
    g_manual_trip_seen_airborne = 0U;

    XM_SetAssistTorqueRH(0.0f);
    XM_SetAssistTorqueLH(0.0f);

    s_usb_debug_tick = XM_GetTick();

    XM_SetLedEffect(XM_LED_1, XM_LED_BLINK, 200);
    XM_SendUsbDebugMessage(
        "[DAMP] ACTIVE: stop = threshold reversal only; no duration timer.\r\n"
    );
}

static void Active_Loop(void)
{
    if (XM.status.h10.h10Mode != XM_H10_MODE_ASSIST) {
        XM_SetAssistTorqueRH(0.0f);
        XM_SetAssistTorqueLH(0.0f);
        XM_TSM_TransitionTo(s_tsm, XM_STATE_STANDBY);
        return;
    }

    uint32_t now = XM_GetTick();

    /* ---------------------------------------------------------------------- */
    /* A. Hip angle -> filtered hip angular velocity                          */
    /* ---------------------------------------------------------------------- */

    g_right_hip_angle_deg = XM.status.h10.rightHipAngle;
    g_left_hip_angle_deg  = XM.status.h10.leftHipAngle;

    g_right_hip_vel_dps =
        _UpdateHipVelocity(&s_rh_vel, g_right_hip_angle_deg, now);

    g_left_hip_vel_dps =
        _UpdateHipVelocity(&s_lh_vel, g_left_hip_angle_deg, now);

    g_right_flex_vel_dps =
        (float)g_right_flexion_sign * g_right_hip_vel_dps;

    g_left_flex_vel_dps =
        (float)g_left_flexion_sign * g_left_hip_vel_dps;

    g_interleg_angle_deg =
        fabsf(g_right_hip_angle_deg - g_left_hip_angle_deg);

    g_angle_scale =
        1.0f + g_angle_gain_per_deg * g_interleg_angle_deg;

    /* ---------------------------------------------------------------------- */
    /* B. Manual perturbation event input                                     */
    /* ---------------------------------------------------------------------- */

    _UpdateManualPerturbationButtons();
    _UpdateManualTripFootContact();

    /* Optional Live-Expression event trigger */
    if (g_test_trigger == 1U) {
        g_test_trigger = 0U;

        DampingAssist_Trigger(
            (PerturbationType_t)g_test_event_type,
            (LegSide_t)g_test_event_side
        );
    }

    /* Optional manual trip foot-contact transition */
    if (g_test_trip_foot_contact == 1U) {
        g_test_trip_foot_contact = 0U;

        DampingAssist_TripFootContact(
            (LegSide_t)g_test_event_side
        );
    }

    /*
     * Optional test COM feed while trip waits for foot contact.
     * This is NOT used when COM scaling is disabled.
     */
    if ((s_ctrl.type == PERT_EARLY_SWING_TRIP) &&
        (s_ctrl.state == CTRL_WAIT_TRIP_FOOT_CONTACT)) {
        DampingAssist_UpdateComVelocity(g_test_com_velocity_mps);
    }

    /* ---------------------------------------------------------------------- */
    /* C. Reaction controller                                                 */
    /* ---------------------------------------------------------------------- */

    float rh_tau = 0.0f;
    float lh_tau = 0.0f;

    _RunReactionController(&rh_tau, &lh_tau);

    /* Raw controller result before master output gate / optional clamp */
    g_right_tau_raw_nm = rh_tau;
    g_left_tau_raw_nm  = lh_tau;

    /* Master arm */
    if (g_control_ON != 1U) {
        rh_tau = 0.0f;
        lh_tau = 0.0f;
    }

    /* Optional clamp */
    if (g_enable_torque_clamp == 1U) {
        rh_tau = _ClampFloat(
            rh_tau,
            -fabsf(g_torque_limit_nm),
            +fabsf(g_torque_limit_nm)
        );

        lh_tau = _ClampFloat(
            lh_tau,
            -fabsf(g_torque_limit_nm),
            +fabsf(g_torque_limit_nm)
        );
    }

    XM_SetAssistTorqueRH(rh_tau);
    XM_SetAssistTorqueLH(lh_tau);

    g_right_tau_cmd_nm = rh_tau;
    g_left_tau_cmd_nm  = lh_tau;

    /* ---------------------------------------------------------------------- */
    /* D. Original 12-float USB monitoring                                    */
    /* ---------------------------------------------------------------------- */

    s_stream.data[0]  = g_right_hip_angle_deg;
    s_stream.data[1]  = g_left_hip_angle_deg;
    s_stream.data[2]  = g_right_hip_vel_dps;
    s_stream.data[3]  = g_left_hip_vel_dps;
    s_stream.data[4]  = g_right_B_eff;
    s_stream.data[5]  = g_left_B_eff;
    s_stream.data[6]  = g_right_tau_raw_nm;
    s_stream.data[7]  = g_left_tau_raw_nm;
    s_stream.data[8]  = (float)g_active_event_type;
    s_stream.data[9]  = (float)g_reaction_state_dbg;
    s_stream.data[10] = g_angle_scale;
    s_stream.data[11] = g_trip_mean_com_velocity_mps;

    XM_SendUsbDataWithId(
        &s_stream,
        sizeof(s_stream),
        USB_STREAM_ID
    );

    _UpdateUsbDebug();
}

static void Active_Exit(void)
{
    XM_SetAssistTorqueRH(0.0f);
    XM_SetAssistTorqueLH(0.0f);

    XM_SetControlMode(XM_CTRL_MONITOR);

    g_right_tau_raw_nm = 0.0f;
    g_left_tau_raw_nm  = 0.0f;
    g_right_tau_cmd_nm = 0.0f;
    g_left_tau_cmd_nm  = 0.0f;

    _ResetReactionController();

    XM_SetLedEffect(XM_LED_1, XM_LED_HEARTBEAT, 1000);
    XM_SendUsbDebugMessage("[DAMP] control stopped; torque released.\r\n");
}

/* ========================================================================== */
/* 11. REACTION CONTROLLER                                                    */
/* ========================================================================== */

static void _StartReaction(PerturbationType_t type, LegSide_t side)
{
    _ResetReactionController();

    s_ctrl.type  = type;
    s_ctrl.side  = side;

    g_active_event_type = (uint16_t)type;
    g_active_event_side = (uint16_t)side;

    s_ctrl.threshold_crossed_first = false;
    g_threshold_first_crossed_dbg = 0U;

    switch (type) {

    case PERT_ES_SLIP:
        /* Harmful primary direction = flexion (+) */
        s_ctrl.primary_harmful_direction = +1;
        s_ctrl.state = CTRL_ACTIVE;
        break;

    case PERT_LS_SLIP:
        /* Harmful primary direction = extension (-) */
        s_ctrl.primary_harmful_direction = -1;
        s_ctrl.state = CTRL_ACTIVE;
        break;

    case PERT_EARLY_SWING_TRIP:
        /*
         * BTN3 marks obstacle contact.
         * Do not damp until recovery-foot contact.
         */
        s_ctrl.primary_harmful_direction = +1;
        s_ctrl.state = CTRL_WAIT_TRIP_FOOT_CONTACT;
        break;

    default:
        _ResetReactionController();
        break;
    }

    g_reaction_state_dbg = (uint16_t)s_ctrl.state;
}

static void _RunReactionController(float *rh_tau, float *lh_tau)
{
    *rh_tau = 0.0f;
    *lh_tau = 0.0f;

    g_right_B_eff = 0.0f;
    g_left_B_eff  = 0.0f;

    if (s_ctrl.state != CTRL_ACTIVE) {
        g_reaction_state_dbg = (uint16_t)s_ctrl.state;
        return;
    }

    float rh_qdot = g_right_hip_vel_dps;
    float lh_qdot = g_left_hip_vel_dps;

    float rh_flex_vel = g_right_flex_vel_dps;
    float lh_flex_vel = g_left_flex_vel_dps;

    LegSide_t primary = s_ctrl.side;
    LegSide_t secondary =
        (primary == LEG_RIGHT) ? LEG_LEFT : LEG_RIGHT;

    float primary_flex_vel =
        (primary == LEG_RIGHT) ? rh_flex_vel : lh_flex_vel;

    float secondary_flex_vel =
        (secondary == LEG_RIGHT) ? rh_flex_vel : lh_flex_vel;

    float B_primary   = 0.0f;
    float B_secondary = 0.0f;

    bool primary_enable   = false;
    bool secondary_enable = false;

    switch (s_ctrl.type) {

    case PERT_ES_SLIP:
        /*
         * Primary / impact leg:
         *   damp flexion only.
         *
         * Secondary / stance leg:
         *   damp extension only.
         */
        B_primary   = g_k_es_impact;
        B_secondary = g_k_es_stance * g_angle_scale;

        primary_enable   = (primary_flex_vel > 0.0f);
        secondary_enable = (secondary_flex_vel < 0.0f);
        break;

    case PERT_LS_SLIP:
        /*
         * Primary / slipping leg:
         *   damp extension only.
         *
         * Secondary / stance leg:
         *   damp flexion only.
         */
        B_primary   = g_k_ls_slip;
        B_secondary = g_k_ls_stance * g_angle_scale;

        primary_enable   = (primary_flex_vel < 0.0f);
        secondary_enable = (secondary_flex_vel > 0.0f);
        break;

    case PERT_EARLY_SWING_TRIP:
        /*
         * This branch is reached only AFTER perturbed-foot ground contact.
         *
         * Primary / perturbed leg:
         *   damp flexion only.
         *
         * Secondary / stance leg:
         *   damp extension only.
         */
        B_primary =
            g_k_trip_impact * s_ctrl.trip_com_scale;

        B_secondary =
            g_k_trip_stance;

        primary_enable   = (primary_flex_vel > 0.0f);
        secondary_enable = (secondary_flex_vel < 0.0f);
        break;

    default:
        _StopReaction();
        return;
    }

    /* ---------------------------------------------------------------------- */
    /* Calculate continuous damping torque                                    */
    /* ---------------------------------------------------------------------- */

    if (primary == LEG_RIGHT) {

        if (primary_enable) {
            g_right_B_eff = B_primary;

            *rh_tau = _ComputeDampingTorqueNm(
                LEG_RIGHT,
                B_primary,
                rh_qdot
            );
        }

        if (secondary_enable) {
            g_left_B_eff = B_secondary;

            *lh_tau = _ComputeDampingTorqueNm(
                LEG_LEFT,
                B_secondary,
                lh_qdot
            );
        }

    } else {

        if (primary_enable) {
            g_left_B_eff = B_primary;

            *lh_tau = _ComputeDampingTorqueNm(
                LEG_LEFT,
                B_primary,
                lh_qdot
            );
        }

        if (secondary_enable) {
            g_right_B_eff = B_secondary;

            *rh_tau = _ComputeDampingTorqueNm(
                LEG_RIGHT,
                B_secondary,
                rh_qdot
            );
        }
    }

    /* ---------------------------------------------------------------------- */
    /* ONLY STOP CONDITION: threshold-confirmed sign reversal                 */
    /* ---------------------------------------------------------------------- */

    if (_PrimaryVelocityReversed()) {

        /*
         * Force both output torques to zero in THIS SAME control-loop cycle.
         */
        *rh_tau = 0.0f;
        *lh_tau = 0.0f;

        g_right_B_eff = 0.0f;
        g_left_B_eff  = 0.0f;

        _StopReaction();
    }

    g_reaction_state_dbg = (uint16_t)s_ctrl.state;
}

/*
 * Threshold-confirmed sign reversal.
 *
 * ES / Trip:
 *
 *     first:  qdot_flex >= +TH
 *     later:  qdot_flex <= -TH   -> STOP
 *
 * LS:
 *
 *     first:  qdot_flex <= -TH
 *     later:  qdot_flex >= +TH   -> STOP
 */
static bool _PrimaryVelocityReversed(void)
{
    float v  = _GetFlexVelDps(s_ctrl.side);
    float th = fabsf(g_reversal_threshold_dps);

    /*
     * Avoid a zero/negative tuning accident.
     * A nonpositive threshold makes reversal detection meaningless.
     */
    if (th <= 0.0f) {
        return false;
    }

    /* ES slip / Trip: harmful direction = flexion (+) */
    if (s_ctrl.primary_harmful_direction > 0) {

        /* FIRST threshold */
        if (!s_ctrl.threshold_crossed_first) {

            if (v >= +th) {
                s_ctrl.threshold_crossed_first = true;
                g_threshold_first_crossed_dbg = 1U;
            }

            return false;
        }

        /* OPPOSITE threshold */
        if (v <= -th) {
            g_last_stop_flex_vel_dps = v;
            return true;
        }

        return false;
    }

    /* LS slip: harmful direction = extension (-) */
    if (s_ctrl.primary_harmful_direction < 0) {

        /* FIRST threshold */
        if (!s_ctrl.threshold_crossed_first) {

            if (v <= -th) {
                s_ctrl.threshold_crossed_first = true;
                g_threshold_first_crossed_dbg = 1U;
            }

            return false;
        }

        /* OPPOSITE threshold */
        if (v >= +th) {
            g_last_stop_flex_vel_dps = v;
            return true;
        }

        return false;
    }

    return false;
}

static void _StopReaction(void)
{
    g_reaction_stop_count++;

    /*
     * No refractory timer.
     * Stop means immediately return to IDLE.
     * Physical buttons are rising-edge triggered, so holding a button does not
     * retrigger the event.
     */
    _ResetReactionController();
}

static void _ResetReactionController(void)
{
    s_ctrl.state = CTRL_IDLE;
    s_ctrl.type  = PERT_NONE;
    s_ctrl.side  = LEG_NONE;

    s_ctrl.com_velocity_sum   = 0.0f;
    s_ctrl.com_velocity_count = 0U;
    s_ctrl.mean_com_velocity_mps = 0.0f;
    s_ctrl.trip_com_scale = 1.0f;

    s_ctrl.primary_harmful_direction = 0;
    s_ctrl.threshold_crossed_first = false;

    g_threshold_first_crossed_dbg = 0U;

    g_active_event_type = PERT_NONE;
    g_active_event_side = LEG_NONE;
    g_reaction_state_dbg = CTRL_IDLE;

    g_right_B_eff = 0.0f;
    g_left_B_eff  = 0.0f;

    g_trip_mean_com_velocity_mps = 0.0f;
    g_trip_com_scale = 1.0f;
}

/* ========================================================================== */
/* 12. DAMPING / KINEMATIC HELPERS                                            */
/* ========================================================================== */

static float _ComputeDampingTorqueNm(
    LegSide_t side,
    float B_eff,
    float qdot_dps
)
{
    float qdot_rad_s =
        qdot_dps * DEG2RAD;

    float tau_encoder_coordinate =
        -B_eff * qdot_rad_s;

    if (side == LEG_RIGHT) {
        return
            (float)g_right_torque_sign *
            tau_encoder_coordinate;
    }

    if (side == LEG_LEFT) {
        return
            (float)g_left_torque_sign *
            tau_encoder_coordinate;
    }

    return 0.0f;
}

static float _GetFlexVelDps(LegSide_t side)
{
    if (side == LEG_RIGHT) {
        return g_right_flex_vel_dps;
    }

    if (side == LEG_LEFT) {
        return g_left_flex_vel_dps;
    }

    return 0.0f;
}

/* ========================================================================== */
/* 13. HIP VELOCITY ESTIMATOR                                                 */
/* ========================================================================== */

static void _ResetVelocityEstimator(HipVelEstimator_t *e)
{
    e->initialized = false;
    e->last_angle_deg = 0.0f;
    e->last_change_tick = 0U;
    e->vel_dps = 0.0f;
}

static float _UpdateHipVelocity(
    HipVelEstimator_t *e,
    float angle_deg,
    uint32_t now
)
{
    if (!e->initialized) {
        e->initialized = true;
        e->last_angle_deg = angle_deg;
        e->last_change_tick = now;
        e->vel_dps = 0.0f;
        return 0.0f;
    }

    float d_angle =
        angle_deg - e->last_angle_deg;

    /*
     * Encoder angle has not changed enough to regard this as a new sample.
     */
    if (fabsf(d_angle) < ANGLE_EPS_DEG) {

        if ((now - e->last_change_tick) > VEL_STALE_TIMEOUT_MS) {
            e->vel_dps = 0.0f;
        }

        return e->vel_dps;
    }

    uint32_t elapsed_ms =
        now - e->last_change_tick;

    if (elapsed_ms == 0U) {
        return e->vel_dps;
    }

    /*
     * If there was an abnormally long sample gap, update the reference angle
     * but do not create a large derivative spike.
     */
    if (elapsed_ms > VEL_MAX_GAP_MS) {
        e->last_angle_deg = angle_deg;
        e->last_change_tick = now;
        return e->vel_dps;
    }

    float dt_s =
        (float)elapsed_ms * 0.001f;

    float raw_vel_dps =
        d_angle / dt_s;

    float alpha =
        _LpfAlpha(g_vel_lpf_hz, dt_s);

    e->vel_dps +=
        alpha * (raw_vel_dps - e->vel_dps);

    e->last_angle_deg = angle_deg;
    e->last_change_tick = now;

    return e->vel_dps;
}

static float _LpfAlpha(float cutoff_hz, float dt_s)
{
    if (cutoff_hz <= 0.0f) {
        return 1.0f;
    }

    float wc_dt =
        TWO_PI_F * cutoff_hz * dt_s;

    return
        wc_dt / (1.0f + wc_dt);
}

/* ========================================================================== */
/* 14. XM PHYSICAL BUTTON MANUAL DETECTOR                                     */
/* ========================================================================== */

static bool _ButtonRisingEdge(
    XmBtnState_t now,
    XmBtnState_t prev
)
{
    return
        (now == XM_PRESSED) &&
        (prev == XM_RELEASED);
}

static bool _GetH10FootContact(LegSide_t side)
{
    if (side == LEG_RIGHT) {
        return
            XM.status.h10.isRightFootContact ?
            true : false;
    }

    if (side == LEG_LEFT) {
        return
            XM.status.h10.isLeftFootContact ?
            true : false;
    }

    return false;
}

static void _UpdateManualPerturbationButtons(void)
{
    XmBtnState_t b1 =
        XM_GetButtonState(XM_BTN_1);

    XmBtnState_t b2 =
        XM_GetButtonState(XM_BTN_2);

    XmBtnState_t b3 =
        XM_GetButtonState(XM_BTN_3);

    g_manual_btn1_state_dbg = (uint16_t)b1;
    g_manual_btn2_state_dbg = (uint16_t)b2;
    g_manual_btn3_state_dbg = (uint16_t)b3;

    LegSide_t side =
        (g_manual_event_side == LEG_LEFT) ?
        LEG_LEFT :
        LEG_RIGHT;

    /* BTN 1 -> ES slip */
    if (_ButtonRisingEdge(b1, s_prevBtn1)) {

        if (s_ctrl.state == CTRL_IDLE) {
            g_manual_last_button = 1U;
            g_manual_trigger_count++;

            DampingAssist_Trigger(
                PERT_ES_SLIP,
                side
            );

            XM_SendUsbDebugMessage(
                "[MANUAL] BTN1 -> ES SLIP\r\n"
            );
        }
    }

    /* BTN 2 -> LS slip */
    if (_ButtonRisingEdge(b2, s_prevBtn2)) {

        if (s_ctrl.state == CTRL_IDLE) {
            g_manual_last_button = 2U;
            g_manual_trigger_count++;

            DampingAssist_Trigger(
                PERT_LS_SLIP,
                side
            );

            XM_SendUsbDebugMessage(
                "[MANUAL] BTN2 -> LS SLIP\r\n"
            );
        }
    }

    /* BTN 3 -> early-swing trip / obstacle contact */
    if (_ButtonRisingEdge(b3, s_prevBtn3)) {

        if (s_ctrl.state == CTRL_IDLE) {
            g_manual_last_button = 3U;
            g_manual_trigger_count++;

            DampingAssist_Trigger(
                PERT_EARLY_SWING_TRIP,
                side
            );

            s_manualTripWaiting = true;

            /*
             * If trigger occurs while the perturbed foot is already in swing,
             * airborne status is immediately confirmed.
             */
            s_manualTripSeenAirborne =
                !_GetH10FootContact(side);

            g_manual_trip_waiting_for_contact = 1U;

            g_manual_trip_seen_airborne =
                s_manualTripSeenAirborne ?
                1U :
                0U;

            XM_SendUsbDebugMessage(
                "[MANUAL] BTN3 -> TRIP; waiting foot contact\r\n"
            );
        }
    }

    s_prevBtn1 = b1;
    s_prevBtn2 = b2;
    s_prevBtn3 = b3;
}

static void _UpdateManualTripFootContact(void)
{
    if (!s_manualTripWaiting) {
        return;
    }

    /*
     * Another path already ended/reset the trip.
     */
    if ((s_ctrl.type != PERT_EARLY_SWING_TRIP) ||
        (s_ctrl.state != CTRL_WAIT_TRIP_FOOT_CONTACT)) {

        s_manualTripWaiting = false;
        g_manual_trip_waiting_for_contact = 0U;
        return;
    }

    /*
     * If automatic H10 contact detection is disabled, wait for
     * g_test_trip_foot_contact = 1.
     */
    if (g_manual_trip_use_h10_foot_contact != 1U) {
        return;
    }

    LegSide_t side =
        s_ctrl.side;

    bool contact =
        _GetH10FootContact(side);

    /*
     * Require at least one airborne sample before accepting ground contact.
     */
    if (!contact) {
        s_manualTripSeenAirborne = true;
        g_manual_trip_seen_airborne = 1U;
        return;
    }

    if (s_manualTripSeenAirborne && contact) {

        DampingAssist_TripFootContact(side);

        s_manualTripWaiting = false;
        g_manual_trip_waiting_for_contact = 0U;

        XM_SendUsbDebugMessage(
            "[MANUAL] TRIP foot contact -> ACTIVE\r\n"
        );
    }
}

/* ========================================================================== */
/* 15. GENERIC HELPERS / DEBUG                                                */
/* ========================================================================== */

static float _ClampFloat(float x, float lo, float hi)
{
    if (x < lo) {
        return lo;
    }

    if (x > hi) {
        return hi;
    }

    return x;
}

static void _UpdateUsbDebug(void)
{
    uint32_t now =
        XM_GetTick();

    if ((now - s_usb_debug_tick) < USB_DEBUG_PERIOD_MS) {
        return;
    }

    s_usb_debug_tick = now;

    char buf[240];

    snprintf(
        buf,
        sizeof(buf),
        "DAMP ON:%u event:%u side:%u state:%u armed:%u "
        "| RH qd:%.1f B:%.2f raw:%.2f cmd:%.2f "
        "| LH qd:%.1f B:%.2f raw:%.2f cmd:%.2f "
        "| TH:%.1f\r\n",
        (unsigned)g_control_ON,
        (unsigned)g_active_event_type,
        (unsigned)g_active_event_side,
        (unsigned)g_reaction_state_dbg,
        (unsigned)g_threshold_first_crossed_dbg,

        g_right_hip_vel_dps,
        g_right_B_eff,
        g_right_tau_raw_nm,
        g_right_tau_cmd_nm,

        g_left_hip_vel_dps,
        g_left_B_eff,
        g_left_tau_raw_nm,
        g_left_tau_cmd_nm,

        g_reversal_threshold_dps
    );

    XM_SendUsbDebugMessage(buf);
}
