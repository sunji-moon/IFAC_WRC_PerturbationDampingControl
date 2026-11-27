/**
 ******************************************************************************
 * @file    user_app.c
 * @brief   Detector-triggered continuous hip damping control for XM10 + H10
 *
 * CONTROL CONCEPT
 * ----------------------------------------------------------------------------
 * The perturbation detector is assumed to already work.
 *
 * While a reaction controller is ACTIVE, torque is recomputed every control
 * loop from the CURRENT hip angular velocity:
 *
 *     tau = - B_eff * qdot
 *
 * This is continuous viscous damping, NOT a latched step torque.
 *
 * ES SLIP
 *   event side = impact / perturbed leg
 *   - impact leg : damp HIP FLEXION
 *   - stance leg : damp HIP EXTENSION
 *                  B_eff = k_es_stance * angle_scale
 *   - timing      : immediately after detection
 *   - duration    : until impact-leg hip velocity reverses direction
 *
 * LS SLIP
 *   event side = slipping leg
 *   - slipping leg : damp HIP EXTENSION
 *   - stance leg   : damp HIP FLEXION
 *                    B_eff = k_ls_stance * angle_scale
 *   - timing       : immediately after detection
 *   - duration     : until slipping-leg hip velocity reverses direction
 *
 * EARLY-SWING TRIP
 *   event side = perturbed / impact leg
 *   - detector trigger = obstacle contact
 *   - WAIT until perturbed foot contacts ground
 *   - perturbed leg : damp HIP FLEXION
 *                     B_eff = k_trip_impact * com_scale
 *   - stance leg    : damp HIP EXTENSION
 *   - duration      : until perturbed-leg hip velocity reverses direction
 *
 * IMPORTANT ABOUT COM
 * ----------------------------------------------------------------------------
 * This file does NOT estimate whole-body COM velocity.
 * During a trip, an external estimator/logger may continuously call:
 *
 *     DampingAssist_UpdateComVelocity(com_velocity_mps);
 *
 * between obstacle contact and recovery-foot ground contact. At foot contact:
 *
 *     DampingAssist_TripFootContact(side);
 *
 * The average COM velocity accumulated over that interval is then used for
 * trip impact-leg damping scaling. Until you implement/validate a COM estimator,
 * keep g_trip_com_scaling_enable = 0 and the trip impact leg simply uses
 * k_trip_impact.
 *
 * TORQUE CLAMP
 * ----------------------------------------------------------------------------
 * The user requested viewing/outputting the torque exactly as calculated.
 * Therefore g_enable_torque_clamp defaults to 0.
 *
 * A clamp implementation remains available for later:
 *     g_enable_torque_clamp = 1
 *     g_torque_limit_nm = ...
 *
 * CAUTION:
 *   With clamp disabled, a large qdot or tuning coefficient can create a large
 *   command. Keep g_control_ON = 0 until bench sign checks are complete.
 ******************************************************************************
 */

#include "xm_api.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

/* ========================================================================== */
/* 1. EVENT TYPES                                                              */
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
    CTRL_WAIT_TRIP_FOOT_CONTACT,
    CTRL_ACTIVE,
    CTRL_REFRACTORY
} ReactionState_t;

/* ========================================================================== */
/* 2. TUNING PARAMETERS — STM32CubeIDE Live Expressions                       */
/* ========================================================================== */

/* Master arm: 0 = never send nonzero torque. */
volatile uint16_t g_control_ON = 0U;

/*
 * Torque output clamp.
 * Requested behavior: OFF, so calculated torque is sent unchanged.
 */
volatile uint16_t g_enable_torque_clamp = 0U;
volatile float    g_torque_limit_nm     = 10.0f;

/*
 * Encoder / torque sign conventions.
 *
 * flexion_sign:
 *   +1 : +encoder velocity means hip flexion
 *   -1 : -encoder velocity means hip flexion
 *
 * torque_sign:
 *   +1 : XM positive torque acts in +encoder-angle direction
 *   -1 : XM positive torque acts in -encoder-angle direction
 *
 * Verify these on the bench before human use.
 */
volatile int16_t g_right_flexion_sign = +1;
volatile int16_t g_left_flexion_sign  = +1;
volatile int16_t g_right_torque_sign  = +1;
volatile int16_t g_left_torque_sign   = +1;

/* ---------- ES slip damping coefficients [Nm*s/rad] ---------- */
volatile float g_k_es_impact = 0.30f;   /* impact leg: damp flexion */
volatile float g_k_es_stance = 0.30f;   /* stance leg: damp extension */

/* ---------- LS slip damping coefficients [Nm*s/rad] ---------- */
volatile float g_k_ls_slip   = 0.30f;   /* slipping leg: damp extension */
volatile float g_k_ls_stance = 0.30f;   /* stance leg: damp flexion */

/* ---------- Early-swing trip damping coefficients [Nm*s/rad] ---------- */
volatile float g_k_trip_impact = 0.30f; /* perturbed leg: damp flexion */
volatile float g_k_trip_stance = 0.30f; /* stance leg: damp extension */

/*
 * Inter-leg angle scaling for slip stance-leg damping.
 *
 *   angle_scale = 1 + g_angle_gain_per_deg * abs(q_R - q_L)
 *
 * Example:
 *   gain = 0.02 /deg, angle difference = 30 deg -> scale = 1.6
 *
 * Set gain = 0 to disable angle scaling.
 */
volatile float g_angle_gain_per_deg = 0.02f;

/*
 * Trip COM scaling:
 *
 *   com_scale = 1 + g_trip_com_gain_per_mps * abs(mean_COM_velocity)
 *
 * Until COM velocity is actually estimated and fed into this file, keep
 * g_trip_com_scaling_enable = 0.
 */
volatile uint16_t g_trip_com_scaling_enable = 0U;
volatile float    g_trip_com_gain_per_mps   = 1.0f;

/*
 * Trip timing:
 * Preferred path:
 *   detector -> WAIT -> DampingAssist_TripFootContact(side) -> ACTIVE
 *
 * Fallback path:
 *   detector -> fixed delay -> ACTIVE
 */
volatile uint16_t g_trip_fallback_enable   = 0U;
volatile uint32_t g_trip_fallback_delay_ms = 220U;

/*
 * Stop / safety timing.
 *
 * Manual-detector bench configuration: stop rule is PURE sign-change
 * reversal in _PrimaryVelocityReversed() (discrete IVT, no magnitude
 * threshold). g_zero_cross_threshold_dps and g_max_active_ms below are left
 * declared (Live Expressions still shows them) but are NOT read by the
 * control loop anymore -- changing them live has no effect. Re-wire
 * g_max_active_ms in the CTRL_ACTIVE case before any unsupervised use; see
 * the CAUTION comment there.
 */
volatile float    g_zero_cross_threshold_dps = 5.0f;   /* unused, see above */
volatile uint32_t g_min_active_before_stop_ms = 20U;
volatile uint32_t g_max_active_ms             = 700U;  /* unused, see above */
volatile uint32_t g_refractory_ms             = 250U;

/* Hip velocity LPF cutoff. */
volatile float g_vel_lpf_hz = 15.0f;

/* ========================================================================== */
/* 3. MANUAL BENCH TEST INPUTS                                                 */
/* ========================================================================== */

/*
 * Example:
 *   g_test_event_type = 1;  // ES slip
 *   g_test_event_side = 1;  // right impact leg
 *   g_test_trigger = 1;
 *
 * For trip:
 *   g_test_event_type = 3;
 *   g_test_event_side = 1;
 *   g_test_trigger = 1;       // obstacle contact
 *   ... wait ...
 *   g_test_trip_foot_contact = 1;
 */
volatile uint16_t g_test_trigger           = 0U;
volatile uint16_t g_test_event_type        = PERT_ES_SLIP;
volatile uint16_t g_test_event_side        = LEG_RIGHT;
volatile uint16_t g_test_trip_foot_contact = 0U;

/* Manual COM velocity input for bench/testing of the trip scaling path. */
volatile float g_test_com_velocity_mps = 0.0f;

/* ========================================================================== */
/* 4. PUBLIC DIAGNOSTICS                                                       */
/* ========================================================================== */

volatile float g_right_hip_angle_deg = 0.0f;
volatile float g_left_hip_angle_deg  = 0.0f;
volatile float g_right_hip_vel_dps   = 0.0f;
volatile float g_left_hip_vel_dps    = 0.0f;

/* Flexion-positive representation, useful when checking event directions. */
volatile float g_right_flex_vel_dps  = 0.0f;
volatile float g_left_flex_vel_dps   = 0.0f;

/* Effective damping coefficients used this cycle. */
volatile float g_right_B_eff = 0.0f;
volatile float g_left_B_eff  = 0.0f;

/* Scaling diagnostics. */
volatile float g_interleg_angle_deg = 0.0f;
volatile float g_angle_scale        = 1.0f;
volatile float g_trip_mean_com_velocity_mps = 0.0f;
volatile float g_trip_com_scale             = 1.0f;

/*
 * Raw torque = exact controller calculation BEFORE optional clamp.
 * Output torque = value actually passed to XM_SetAssistTorque*().
 */
volatile float g_right_tau_raw_nm = 0.0f;
volatile float g_left_tau_raw_nm  = 0.0f;
volatile float g_right_tau_cmd_nm = 0.0f;
volatile float g_left_tau_cmd_nm  = 0.0f;

volatile uint16_t g_active_event_type = PERT_NONE;
volatile uint16_t g_active_event_side = LEG_NONE;
volatile uint16_t g_reaction_state_dbg = CTRL_IDLE;
volatile uint32_t g_active_elapsed_ms = 0U;

/* ========================================================================== */
/* 5. PRIVATE DEFINITIONS                                                      */
/* ========================================================================== */

#define DEG2RAD                 0.01745329251994329577f
#define TWO_PI_F                6.28318530717958647692f
#define ANGLE_EPS_DEG           0.005f
#define VEL_STALE_TIMEOUT_MS    120U
#define VEL_MAX_GAP_MS          60U

#define USB_STREAM_ID           0xF0U
#define USB_DEBUG_PERIOD_MS     200U

typedef struct {
    bool     initialized;
    float    last_angle_deg;
    uint32_t last_change_tick;
    float    vel_dps;
} HipVelEstimator_t;

typedef struct {
    ReactionState_t state;
    PerturbationType_t type;
    LegSide_t side;

    uint32_t trigger_tick;
    uint32_t active_tick;
    uint32_t state_tick;

    /* Trip COM accumulation between obstacle contact and foot contact. */
    float    com_velocity_sum;
    uint32_t com_velocity_count;
    float    mean_com_velocity_mps;
    float    trip_com_scale;

    /* Expected primary harmful direction in flexion-positive coordinates:
     * +1 = flexion, -1 = extension.
     */
    int8_t primary_harmful_direction;

    /* Previous-tick primary flex velocity, for pure sign-change (discrete
     * Intermediate Value Theorem) reversal detection -- see
     * _PrimaryVelocityReversed(). prev_valid is false until the first
     * post-min-active-period sample has been recorded. */
    float prev_primary_flex_vel;
    bool  prev_valid;
} ReactionController_t;

typedef struct {
    float data[14];
    /*
     * [0] RH angle deg
     * [1] LH angle deg
     * [2] RH qdot dps
     * [3] LH qdot dps
     * [4] RH B_eff
     * [5] LH B_eff
     * [6] RH tau raw Nm     (controller's raw computed tau, BEFORE
     *                        g_control_ON gate / optional clamp)
     * [7] LH tau raw Nm
     * [8] event
     * [9] state
     * [10] angle scale
     * [11] mean COM velocity
     * [12] RH tau cmd Nm    (what actually went to XM_SetAssistTorqueRH() --
     *                        AFTER the g_control_ON gate and optional clamp;
     *                        this is the REAL sent torque, 0 whenever
     *                        g_control_ON=0 even if [6] is nonzero)
     * [13] LH tau cmd Nm
     */
} StreamData_t;

/* ========================================================================== */
/* 6. STATIC DATA                                                              */
/* ========================================================================== */

static XmTsmHandle_t s_tsm;
static HipVelEstimator_t s_rh_vel;
static HipVelEstimator_t s_lh_vel;
static ReactionController_t s_ctrl;
static StreamData_t s_stream;
static uint32_t s_usb_debug_tick = 0U;

/* ========================================================================== */
/* 7. PROTOTYPES                                                               */
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
static void _StartReaction(PerturbationType_t type, LegSide_t side, uint32_t now);
static void _RunReactionController(uint32_t now, float *rh_tau, float *lh_tau);
static void _StopReaction(uint32_t now);

static float _GetFlexVelDps(LegSide_t side);
static float _GetHipVelDps(LegSide_t side);
static float _GetHipAngleDeg(LegSide_t side);
static float _ComputeDampingTorqueNm(LegSide_t side, float B_eff, float qdot_dps);
static bool  _PrimaryVelocityReversed(void);

static float _ClampFloat(float x, float lo, float hi);
static void  _UpdateUsbDebug(void);

/* ========================================================================== */
/* 8. PUBLIC DETECTOR / SENSOR INTERFACE                                       */
/* ========================================================================== */

/**
 * @brief Existing detector calls this once at perturbation detection.
 *
 * ES slip:
 *   side = impact / perturbed leg. Reaction starts immediately.
 *
 * LS slip:
 *   side = slipping leg. Reaction starts immediately.
 *
 * Early-swing trip:
 *   side = perturbed leg. This marks OBSTACLE CONTACT.
 *   Controller waits for DampingAssist_TripFootContact().
 */
void DampingAssist_Trigger(PerturbationType_t type, LegSide_t side)
{
    uint32_t now = XM_GetTick();

    if ((type == PERT_NONE) || (side == LEG_NONE)) {
        return;
    }

    if (s_ctrl.state != CTRL_IDLE) {
        return;
    }

    _StartReaction(type, side, now);
}

/**
 * @brief Feed estimated COM forward velocity during a pending trip.
 *
 * Call every loop (or whenever a new estimate is available) from obstacle
 * contact until recovery-foot ground contact.
 */
void DampingAssist_UpdateComVelocity(float com_velocity_mps)
{
    if ((s_ctrl.type != PERT_EARLY_SWING_TRIP) ||
        (s_ctrl.state != CTRL_WAIT_TRIP_FOOT_CONTACT)) {
        return;
    }

    s_ctrl.com_velocity_sum += com_velocity_mps;
    s_ctrl.com_velocity_count++;
}

/**
 * @brief Call when the perturbed trip foot contacts the ground.
 */
void DampingAssist_TripFootContact(LegSide_t side)
{
    uint32_t now = XM_GetTick();

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
            1.0f
            + g_trip_com_gain_per_mps
            * fabsf(s_ctrl.mean_com_velocity_mps);
    } else {
        s_ctrl.trip_com_scale = 1.0f;
    }

    g_trip_mean_com_velocity_mps = s_ctrl.mean_com_velocity_mps;
    g_trip_com_scale             = s_ctrl.trip_com_scale;

    s_ctrl.state = CTRL_ACTIVE;
    s_ctrl.active_tick = now;
    s_ctrl.state_tick = now;
}

/* ========================================================================== */
/* 9. XM ENTRY POINTS                                                          */
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
        "{\"name\":\"mean_COM_v\",\"unit\":\"m/s\"},"
        "{\"name\":\"RH_tau_cmd\",\"unit\":\"Nm\"},"
        "{\"name\":\"LH_tau_cmd\",\"unit\":\"Nm\"}]"
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
/* 10. TSM CALLBACKS                                                          */
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

    XM_SetAssistTorqueRH(0.0f);
    XM_SetAssistTorqueLH(0.0f);

    s_usb_debug_tick = XM_GetTick();

    XM_SetLedEffect(XM_LED_1, XM_LED_BLINK, 200);
    XM_SendUsbDebugMessage(
        "[DAMP] ACTIVE. Clamp default OFF; g_control_ON default OFF.\r\n"
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
    /* A. H10 hip encoder -> filtered qdot                                    */
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
    /* B. Manual bench hooks                                                  */
    /* ---------------------------------------------------------------------- */
    if (g_test_trigger == 1U) {
        g_test_trigger = 0U;
        DampingAssist_Trigger(
            (PerturbationType_t)g_test_event_type,
            (LegSide_t)g_test_event_side
        );
    }

    if (g_test_trip_foot_contact == 1U) {
        g_test_trip_foot_contact = 0U;
        DampingAssist_TripFootContact((LegSide_t)g_test_event_side);
    }

    /* Optional manual COM feed for testing the trip path. */
    if ((s_ctrl.type == PERT_EARLY_SWING_TRIP) &&
        (s_ctrl.state == CTRL_WAIT_TRIP_FOOT_CONTACT)) {
        DampingAssist_UpdateComVelocity(g_test_com_velocity_mps);
    }

    /* ---------------------------------------------------------------------- */
    /* C. Reaction controller                                                 */
    /* ---------------------------------------------------------------------- */
    float rh_tau = 0.0f;
    float lh_tau = 0.0f;

    _RunReactionController(now, &rh_tau, &lh_tau);

    g_right_tau_raw_nm = rh_tau;
    g_left_tau_raw_nm  = lh_tau;

    /* Master arm. */
    if (g_control_ON != 1U) {
        rh_tau = 0.0f;
        lh_tau = 0.0f;
    }

    /*
     * Optional clamp. DEFAULT OFF per requested behavior.
     * When OFF, the exact calculated controller torque is sent.
     */
    if (g_enable_torque_clamp == 1U) {
        rh_tau = _ClampFloat(rh_tau, -g_torque_limit_nm, g_torque_limit_nm);
        lh_tau = _ClampFloat(lh_tau, -g_torque_limit_nm, g_torque_limit_nm);
    }

    XM_SetAssistTorqueRH(rh_tau);
    XM_SetAssistTorqueLH(lh_tau);

    g_right_tau_cmd_nm = rh_tau;
    g_left_tau_cmd_nm  = lh_tau;

    /* ---------------------------------------------------------------------- */
    /* D. Monitoring                                                          */
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
    s_stream.data[12] = g_right_tau_cmd_nm;
    s_stream.data[13] = g_left_tau_cmd_nm;

    XM_SendUsbDataWithId(&s_stream, sizeof(s_stream), USB_STREAM_ID);
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

static void _StartReaction(PerturbationType_t type,
                           LegSide_t side,
                           uint32_t now)
{
    _ResetReactionController();

    s_ctrl.type = type;
    s_ctrl.side = side;
    s_ctrl.trigger_tick = now;
    s_ctrl.state_tick = now;

    g_active_event_type = (uint16_t)type;
    g_active_event_side = (uint16_t)side;

    switch (type) {
    case PERT_ES_SLIP:
        /* Primary harmful motion = flexion. */
        s_ctrl.primary_harmful_direction = +1;
        s_ctrl.state = CTRL_ACTIVE;
        s_ctrl.active_tick = now;
        break;

    case PERT_LS_SLIP:
        /* Primary harmful motion = extension. */
        s_ctrl.primary_harmful_direction = -1;
        s_ctrl.state = CTRL_ACTIVE;
        s_ctrl.active_tick = now;
        break;

    case PERT_EARLY_SWING_TRIP:
        /*
         * Obstacle contact only. Do not damp yet.
         * Wait for perturbed foot to contact ground.
         */
        s_ctrl.primary_harmful_direction = +1; /* then damp flexion */
        s_ctrl.state = CTRL_WAIT_TRIP_FOOT_CONTACT;
        break;

    default:
        _ResetReactionController();
        break;
    }

    g_reaction_state_dbg = (uint16_t)s_ctrl.state;
}

static void _RunReactionController(uint32_t now,
                                   float *rh_tau,
                                   float *lh_tau)
{
    *rh_tau = 0.0f;
    *lh_tau = 0.0f;

    g_right_B_eff = 0.0f;
    g_left_B_eff  = 0.0f;
    g_active_elapsed_ms = 0U;

    switch (s_ctrl.state) {

    case CTRL_IDLE:
        break;

    case CTRL_WAIT_TRIP_FOOT_CONTACT:
        /*
         * Preferred timing: explicit ground-contact callback.
         * Optional fallback: manually tuned delay.
         */
        if ((g_trip_fallback_enable == 1U) &&
            ((now - s_ctrl.trigger_tick) >= g_trip_fallback_delay_ms)) {

            if (s_ctrl.com_velocity_count > 0U) {
                s_ctrl.mean_com_velocity_mps =
                    s_ctrl.com_velocity_sum / (float)s_ctrl.com_velocity_count;
            } else {
                s_ctrl.mean_com_velocity_mps = 0.0f;
            }

            if (g_trip_com_scaling_enable == 1U) {
                s_ctrl.trip_com_scale =
                    1.0f
                    + g_trip_com_gain_per_mps
                    * fabsf(s_ctrl.mean_com_velocity_mps);
            } else {
                s_ctrl.trip_com_scale = 1.0f;
            }

            g_trip_mean_com_velocity_mps = s_ctrl.mean_com_velocity_mps;
            g_trip_com_scale = s_ctrl.trip_com_scale;

            s_ctrl.state = CTRL_ACTIVE;
            s_ctrl.active_tick = now;
            s_ctrl.state_tick = now;
        }
        break;

    case CTRL_ACTIVE:
    {
        uint32_t elapsed = now - s_ctrl.active_tick;
        g_active_elapsed_ms = elapsed;

        /*
         * g_max_active_ms fallback intentionally NOT used here (manual-
         * detector bench configuration): stop relies purely on
         * _PrimaryVelocityReversed()'s sign-change test below.
         *
         * CAUTION: without a duration ceiling, if the sign never cleanly
         * flips right after trigger (observed for real trip/slip events --
         * see ND_5 analysis), ACTIVE now only ends at the NEXT natural
         * gait-cycle zero-crossing, not within a bounded time. Acceptable
         * for supervised bench testing; put a timeout back before any
         * unsupervised use.
         */

        /* Current kinematics. */
        float rh_qdot = g_right_hip_vel_dps;
        float lh_qdot = g_left_hip_vel_dps;

        float rh_flex_vel = g_right_flex_vel_dps;
        float lh_flex_vel = g_left_flex_vel_dps;

        LegSide_t primary   = s_ctrl.side;
        LegSide_t secondary = (primary == LEG_RIGHT) ? LEG_LEFT : LEG_RIGHT;

        float primary_flex_vel =
            (primary == LEG_RIGHT) ? rh_flex_vel : lh_flex_vel;
        float secondary_flex_vel =
            (secondary == LEG_RIGHT) ? rh_flex_vel : lh_flex_vel;

        float B_primary = 0.0f;
        float B_secondary = 0.0f;

        bool primary_enable = false;
        bool secondary_enable = false;

        switch (s_ctrl.type) {

        case PERT_ES_SLIP:
            /*
             * Primary = impact leg.
             * Impact leg: damp flexion only.
             * Stance leg: damp extension only.
             */
            B_primary = g_k_es_impact;
            B_secondary = g_k_es_stance * g_angle_scale;

            primary_enable   = (primary_flex_vel > 0.0f);
            secondary_enable = (secondary_flex_vel < 0.0f);
            break;

        case PERT_LS_SLIP:
            /*
             * Primary = slipping leg.
             * Slipping leg: damp extension only.
             * Stance leg: damp flexion only.
             */
            B_primary = g_k_ls_slip;
            B_secondary = g_k_ls_stance * g_angle_scale;

            primary_enable   = (primary_flex_vel < 0.0f);
            secondary_enable = (secondary_flex_vel > 0.0f);
            break;

        case PERT_EARLY_SWING_TRIP:
            /*
             * This state is entered AFTER recovery-foot ground contact.
             * Primary = perturbed leg: damp flexion.
             * Stance leg: damp extension.
             */
            B_primary = g_k_trip_impact * s_ctrl.trip_com_scale;
            B_secondary = g_k_trip_stance;

            primary_enable   = (primary_flex_vel > 0.0f);
            secondary_enable = (secondary_flex_vel < 0.0f);
            break;

        default:
            _StopReaction(now);
            break;
        }

        /*
         * Assign B_eff by physical side for monitoring.
         * Torque itself is still based on that side's current qdot.
         */
        if (primary == LEG_RIGHT) {
            if (primary_enable) {
                g_right_B_eff = B_primary;
                *rh_tau = _ComputeDampingTorqueNm(
                    LEG_RIGHT, B_primary, rh_qdot);
            }
            if (secondary_enable) {
                g_left_B_eff = B_secondary;
                *lh_tau = _ComputeDampingTorqueNm(
                    LEG_LEFT, B_secondary, lh_qdot);
            }
        } else {
            if (primary_enable) {
                g_left_B_eff = B_primary;
                *lh_tau = _ComputeDampingTorqueNm(
                    LEG_LEFT, B_primary, lh_qdot);
            }
            if (secondary_enable) {
                g_right_B_eff = B_secondary;
                *rh_tau = _ComputeDampingTorqueNm(
                    LEG_RIGHT, B_secondary, rh_qdot);
            }
        }

        /*
         * Duration = until primary hip velocity reverses direction.
         * Require a small minimum active period to avoid instant stop due to
         * trigger-time noise.
         */
        if ((elapsed >= g_min_active_before_stop_ms) &&
            _PrimaryVelocityReversed()) {
            _StopReaction(now);
        }
        break;
    }

    case CTRL_REFRACTORY:
        if ((now - s_ctrl.state_tick) >= g_refractory_ms) {
            _ResetReactionController();
        }
        break;

    default:
        _ResetReactionController();
        break;
    }

    g_reaction_state_dbg = (uint16_t)s_ctrl.state;
}

/**
 * @brief Pure sign-change reversal test (discrete Intermediate Value
 *        Theorem), replacing the old magnitude-threshold crossing check.
 *
 * IVT: hip velocity is a continuous function of time; if two consecutive
 * SAMPLES of it have strictly opposite sign, a true zero-crossing (direction
 * reversal) occurred somewhere between those two samples. No
 * g_zero_cross_threshold_dps margin is used -- any sign flip counts.
 *
 * Sampling interval = one control-loop tick (XM_MAIN_LOOP_FREQ_HZ = 1 kHz,
 * i.e. 1 ms), since that is the finest interval at which
 * g_right/left_hip_vel_dps actually receives a new estimate from
 * _UpdateHipVelocity() -- this function is called once per
 * _RunReactionController() tick, i.e. once per control loop.
 *
 * The very first call after the min-active guard opens has no prior sample
 * (prev_valid == false): it only records the baseline and reports no
 * reversal, so a real flip can't be detected until the tick after that.
 */
static bool _PrimaryVelocityReversed(void)
{
    float current = _GetFlexVelDps(s_ctrl.side);
    bool reversed = false;

    if (s_ctrl.prev_valid) {
        reversed = (s_ctrl.prev_primary_flex_vel * current) < 0.0f;
    }

    s_ctrl.prev_primary_flex_vel = current;
    s_ctrl.prev_valid = true;

    return reversed;
}

static void _StopReaction(uint32_t now)
{
    s_ctrl.state = CTRL_REFRACTORY;
    s_ctrl.state_tick = now;

    g_right_B_eff = 0.0f;
    g_left_B_eff  = 0.0f;
}

static void _ResetReactionController(void)
{
    s_ctrl.state = CTRL_IDLE;
    s_ctrl.type  = PERT_NONE;
    s_ctrl.side  = LEG_NONE;

    s_ctrl.trigger_tick = 0U;
    s_ctrl.active_tick  = 0U;
    s_ctrl.state_tick   = 0U;

    s_ctrl.com_velocity_sum   = 0.0f;
    s_ctrl.com_velocity_count = 0U;
    s_ctrl.mean_com_velocity_mps = 0.0f;
    s_ctrl.trip_com_scale = 1.0f;

    s_ctrl.primary_harmful_direction = 0;

    s_ctrl.prev_primary_flex_vel = 0.0f;
    s_ctrl.prev_valid = false;

    g_active_event_type = PERT_NONE;
    g_active_event_side = LEG_NONE;
    g_reaction_state_dbg = CTRL_IDLE;
    g_active_elapsed_ms = 0U;

    g_right_B_eff = 0.0f;
    g_left_B_eff  = 0.0f;

    g_trip_mean_com_velocity_mps = 0.0f;
    g_trip_com_scale = 1.0f;
}

/* ========================================================================== */
/* 12. DAMPING / KINEMATIC HELPERS                                            */
/* ========================================================================== */

/**
 * @brief Calculate the damping torque exactly from the current qdot.
 *
 * tau_in_encoder_coordinate = -B_eff * qdot
 *
 * torque_sign then maps that coordinate into the H10 command sign convention.
 *
 * NOTE: There is NO clamp in this function.
 */
static float _ComputeDampingTorqueNm(LegSide_t side,
                                     float B_eff,
                                     float qdot_dps)
{
    float qdot_rad_s = qdot_dps * DEG2RAD;
    float tau_encoder_coordinate = -B_eff * qdot_rad_s;

    if (side == LEG_RIGHT) {
        return (float)g_right_torque_sign * tau_encoder_coordinate;
    }

    if (side == LEG_LEFT) {
        return (float)g_left_torque_sign * tau_encoder_coordinate;
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

static float _GetHipVelDps(LegSide_t side)
{
    if (side == LEG_RIGHT) {
        return g_right_hip_vel_dps;
    }
    if (side == LEG_LEFT) {
        return g_left_hip_vel_dps;
    }
    return 0.0f;
}

static float _GetHipAngleDeg(LegSide_t side)
{
    if (side == LEG_RIGHT) {
        return g_right_hip_angle_deg;
    }
    if (side == LEG_LEFT) {
        return g_left_hip_angle_deg;
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

static float _UpdateHipVelocity(HipVelEstimator_t *e,
                                float angle_deg,
                                uint32_t now)
{
    if (!e->initialized) {
        e->initialized = true;
        e->last_angle_deg = angle_deg;
        e->last_change_tick = now;
        e->vel_dps = 0.0f;
        return 0.0f;
    }

    float d_angle = angle_deg - e->last_angle_deg;

    if (fabsf(d_angle) < ANGLE_EPS_DEG) {
        if ((now - e->last_change_tick) > VEL_STALE_TIMEOUT_MS) {
            e->vel_dps = 0.0f;
        }
        return e->vel_dps;
    }

    uint32_t elapsed_ms = now - e->last_change_tick;

    if (elapsed_ms == 0U) {
        return e->vel_dps;
    }

    if (elapsed_ms > VEL_MAX_GAP_MS) {
        e->last_angle_deg = angle_deg;
        e->last_change_tick = now;
        return e->vel_dps;
    }

    float dt_s = (float)elapsed_ms * 0.001f;
    float raw_vel_dps = d_angle / dt_s;
    float alpha = _LpfAlpha(g_vel_lpf_hz, dt_s);

    e->vel_dps += alpha * (raw_vel_dps - e->vel_dps);

    e->last_angle_deg = angle_deg;
    e->last_change_tick = now;

    return e->vel_dps;
}

static float _LpfAlpha(float cutoff_hz, float dt_s)
{
    if (cutoff_hz <= 0.0f) {
        return 1.0f;
    }

    float wc_dt = TWO_PI_F * cutoff_hz * dt_s;
    return wc_dt / (1.0f + wc_dt);
}

/* ========================================================================== */
/* 14. GENERIC HELPERS / DEBUG                                                */
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
    uint32_t now = XM_GetTick();

    if ((now - s_usb_debug_tick) < USB_DEBUG_PERIOD_MS) {
        return;
    }

    s_usb_debug_tick = now;

    char buf[220];
    snprintf(
        buf,
        sizeof(buf),
        "DAMP ON:%u clamp:%u event:%u side:%u state:%u "
        "| RH qd:%.1f B:%.3f raw:%.2f cmd:%.2f "
        "| LH qd:%.1f B:%.3f raw:%.2f cmd:%.2f "
        "| angSc:%.2f COM:%.3f\r\n",
        (unsigned)g_control_ON,
        (unsigned)g_enable_torque_clamp,
        (unsigned)g_active_event_type,
        (unsigned)g_active_event_side,
        (unsigned)g_reaction_state_dbg,
        g_right_hip_vel_dps,
        g_right_B_eff,
        g_right_tau_raw_nm,
        g_right_tau_cmd_nm,
        g_left_hip_vel_dps,
        g_left_B_eff,
        g_left_tau_raw_nm,
        g_left_tau_cmd_nm,
        g_angle_scale,
        g_trip_mean_com_velocity_mps
    );

    XM_SendUsbDebugMessage(buf);
}