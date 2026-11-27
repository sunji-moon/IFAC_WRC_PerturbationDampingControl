/**
0826 ES control code with rectified torque
 ******************************************************************************
 * @file    damping_assist_manual_buttons_SJ.c
 * @brief   Manual-button-triggered continuous hip damping control for XM10 + H10
 *
 * IMPORTANT
 * ----------------------------------------------------------------------------
 * This is a PURE C source file. It contains no Python patch/generator code.
 *
 * Manual perturbation trigger (button press / rising edge):
 *   XM BTN_1 press -> ES slip
 *   XM BTN_2 press -> LS slip
 *   XM BTN_3 press -> early-swing trip
 *
 * Long presses are logged as markers but never trigger damping.
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
 * Before the primary-leg velocity peak, the existing velocity is used for
 * torque. After the peak, a separate 4 Hz LPF velocity is used for torque.
 *
 * STOP RULE -- SIGN CHANGE AFTER THE VELOCITY PEAK
 * ----------------------------------------------------------------------------
 * There is NO min-active duration and NO max-active duration in this version.
 * After the harmful-direction velocity peak, the controller stops when the
 * primary flexion-positive velocity changes sign between two nonzero samples.
 *
 * ES slip / early-swing trip:
 *   controller stops when primary flexion-positive hip velocity remains
 *          qdot_flex changes from positive to negative.
 *
 * LS slip:
 *   controller stops when primary flexion-positive hip velocity remains
 *          qdot_flex changes from negative to positive.
 *
 * The previous threshold variables are retained for logger compatibility but
 * are no longer used as the controller stop condition.
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
 * A named, fixed-layout 0xF0 packet is sent at 100 Hz for the focused HND
 * laptop logger. It includes timing/integrity counters, H10 status, latched
 * button markers, controller state, kinematics, torque, and tuning values.
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
volatile uint16_t g_control_ON = 1U;

/* Optional software clamp. OFF by default. */
volatile uint16_t g_enable_torque_clamp = 1U;
volatile float    g_torque_limit_nm     = 12.0f;

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
volatile float g_k_es_impact   = 18.0f;
volatile float g_k_es_stance   = 12.0f;

volatile float g_k_ls_slip     = 12.0f;
volatile float g_k_ls_stance   = 2.8f;

volatile float g_k_trip_impact = 4.0f;
volatile float g_k_trip_stance = 4.0f;

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

/* Latched button marker diagnostics for lossless PC-side event detection. */
volatile uint32_t g_button_marker_sequence   = 0U;
volatile uint32_t g_button_marker_tick_ms    = 0U;
volatile uint16_t g_button_marker_code       = 0U;
volatile uint16_t g_button_marker_kind       = 0U;
volatile uint16_t g_button_marker_accepted   = 0U;
volatile uint16_t g_button_marker_event_type = PERT_NONE;
volatile uint16_t g_button_marker_event_side = LEG_NONE;

/* USB acquisition integrity diagnostics. */
volatile uint32_t g_daq_robot_frame_sequence = 0U;
volatile uint32_t g_daq_user_loop_count       = 0U;
volatile uint32_t g_daq_usb_tx_fail_count     = 0U;
volatile uint32_t g_daq_loop_tick_gap_count   = 0U;

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

#define DAMPING_PROTOCOL_VERSION       1.0f
#define DAMPING_BUILD_ID               260825.0f
#define USB_STREAM_ID                  0xF0U
#define USB_STREAM_PERIOD_MS           10U
#define USB_DEBUG_PERIOD_MS            200U
#define H10_CURRENT_TO_TORQUE_NM_A     1.59375f
#define TORQUE_VEL_LPF_HZ              4.0f

#define BUTTON_MARKER_CLICK            1U
#define BUTTON_MARKER_LONG_PRESS       2U

typedef struct {
    bool     initialized;
    float    last_angle_deg;
    uint32_t last_change_tick;
    float    vel_dps;
} HipVelEstimator_t;

typedef struct {
    float positive_envelope_dps;
    float negative_envelope_dps;
    float signed_velocity_dps;
} EsVelocityEnvelope_t;

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
    bool peak_reached;
    float peak_tracking_velocity_dps;
    float previous_nonzero_velocity_dps;
    bool previous_nonzero_velocity_valid;
} ReactionController_t;

/* Keep this order synchronized with DAMPING_LABELS in the HND logger. */
#define DAMPING_STREAM_CHANNELS(FIRST, NEXT)                                    \
    FIRST(protocol_version,          DAMPING_PROTOCOL_VERSION)                  \
    NEXT (build_id,                  DAMPING_BUILD_ID)                          \
    NEXT (mcu_time_ms,               s_now_tick)                               \
    NEXT (robot_frame_sequence,      g_daq_robot_frame_sequence)               \
    NEXT (user_loop_count,           g_daq_user_loop_count)                    \
    NEXT (usb_tx_fail_count,         g_daq_usb_tx_fail_count)                  \
    NEXT (loop_tick_gap_count,       g_daq_loop_tick_gap_count)                \
    NEXT (usb_connected,             XM_IsUsbStreamConnected())                \
    NEXT (usb_streaming_active,      XM_IsUsbStreamingActive())                \
    NEXT (h10_is_connected,          XM.status.h10.is_connected)               \
    NEXT (h10_mode,                  XM.status.h10.h10Mode)                     \
    NEXT (h10_assist_level,          XM.status.h10.h10AssistLevel)              \
    NEXT (h10_fsm_state,             XM.status.h10.h10FSMcurrentState)          \
    NEXT (xm_control_mode,           XM.command.control_mode)                   \
    NEXT (control_on,                g_control_ON)                              \
    NEXT (torque_clamp_enabled,      g_enable_torque_clamp)                    \
    NEXT (torque_limit_nm,           g_torque_limit_nm)                        \
    NEXT (manual_event_side,         g_manual_event_side)                      \
    NEXT (button1_state,             g_manual_btn1_state_dbg)                  \
    NEXT (button2_state,             g_manual_btn2_state_dbg)                  \
    NEXT (button3_state,             g_manual_btn3_state_dbg)                  \
    NEXT (marker_sequence,           g_button_marker_sequence)                 \
    NEXT (marker_code,               g_button_marker_code)                     \
    NEXT (marker_kind,               g_button_marker_kind)                     \
    NEXT (marker_tick_ms,            g_button_marker_tick_ms)                  \
    NEXT (marker_accepted,           g_button_marker_accepted)                 \
    NEXT (marker_event_type,         g_button_marker_event_type)               \
    NEXT (marker_event_side,         g_button_marker_event_side)               \
    NEXT (manual_trigger_count,      g_manual_trigger_count)                   \
    NEXT (active_event_type,         g_active_event_type)                      \
    NEXT (active_event_side,         g_active_event_side)                      \
    NEXT (reaction_state,            g_reaction_state_dbg)                     \
    NEXT (threshold_first_crossed,   g_threshold_first_crossed_dbg)            \
    NEXT (reaction_stop_count,       g_reaction_stop_count)                    \
    NEXT (last_stop_flex_vel_dps,    g_last_stop_flex_vel_dps)                 \
    NEXT (trip_waiting_for_contact,  g_manual_trip_waiting_for_contact)        \
    NEXT (trip_seen_airborne,        g_manual_trip_seen_airborne)              \
    NEXT (left_foot_contact,         XM.status.h10.isLeftFootContact)          \
    NEXT (right_foot_contact,        XM.status.h10.isRightFootContact)         \
    NEXT (left_hip_angle_deg,        XM.status.h10.leftHipAngle)               \
    NEXT (right_hip_angle_deg,       XM.status.h10.rightHipAngle)              \
    NEXT (left_hip_velocity_dps,     g_left_hip_vel_dps)                       \
    NEXT (right_hip_velocity_dps,    g_right_hip_vel_dps)                      \
    NEXT (left_flex_velocity_dps,    g_left_flex_vel_dps)                      \
    NEXT (right_flex_velocity_dps,   g_right_flex_vel_dps)                     \
    NEXT (interleg_angle_deg,        g_interleg_angle_deg)                     \
    NEXT (left_b_eff_nms_rad,        g_left_B_eff)                             \
    NEXT (right_b_eff_nms_rad,       g_right_B_eff)                            \
    NEXT (left_tau_raw_nm,           g_left_tau_raw_nm)                        \
    NEXT (right_tau_raw_nm,          g_right_tau_raw_nm)                       \
    NEXT (left_tau_cmd_nm,           g_left_tau_cmd_nm)                        \
    NEXT (right_tau_cmd_nm,          g_right_tau_cmd_nm)                       \
    NEXT (left_motor_current_a,      XM.status.h10.leftHipTorque)              \
    NEXT (right_motor_current_a,     XM.status.h10.rightHipTorque)             \
    NEXT (left_joint_torque_est_nm,                                           \
          XM.status.h10.leftHipTorque * H10_CURRENT_TO_TORQUE_NM_A)            \
    NEXT (right_joint_torque_est_nm,                                          \
          XM.status.h10.rightHipTorque * H10_CURRENT_TO_TORQUE_NM_A)           \
    NEXT (angle_scale,               g_angle_scale)                            \
    NEXT (trip_mean_com_velocity_mps,g_trip_mean_com_velocity_mps)             \
    NEXT (trip_com_scale,            g_trip_com_scale)                         \
    NEXT (k_es_impact_nms_rad,       g_k_es_impact)                            \
    NEXT (k_es_stance_nms_rad,       g_k_es_stance)                            \
    NEXT (k_ls_slip_nms_rad,         g_k_ls_slip)                              \
    NEXT (k_ls_stance_nms_rad,       g_k_ls_stance)                            \
    NEXT (k_trip_impact_nms_rad,     g_k_trip_impact)                          \
    NEXT (k_trip_stance_nms_rad,     g_k_trip_stance)                          \
    NEXT (angle_gain_per_deg,        g_angle_gain_per_deg)                     \
    NEXT (trip_com_scaling_enabled,  g_trip_com_scaling_enable)                \
    NEXT (trip_com_gain_per_mps,     g_trip_com_gain_per_mps)                  \
    NEXT (reversal_threshold_dps,    g_reversal_threshold_dps)                 \
    NEXT (velocity_lpf_hz,           g_vel_lpf_hz)

#define DECLARE_STREAM_FIELD(name, source) float name;
typedef struct {
    DAMPING_STREAM_CHANNELS(DECLARE_STREAM_FIELD, DECLARE_STREAM_FIELD)
} DampingStreamPacket_t;
#undef DECLARE_STREAM_FIELD

#define COUNT_STREAM_FIELD(name, source) + 1
enum {
    DAMPING_STREAM_FIELD_COUNT =
        0 DAMPING_STREAM_CHANNELS(COUNT_STREAM_FIELD, COUNT_STREAM_FIELD)
};
#undef COUNT_STREAM_FIELD

_Static_assert(sizeof(DampingStreamPacket_t) ==
                   DAMPING_STREAM_FIELD_COUNT * sizeof(float),
               "Unexpected damping stream packet padding");

/* ========================================================================== */
/* 6. STATIC DATA                                                             */
/* ========================================================================== */

static XmTsmHandle_t s_tsm;

static HipVelEstimator_t s_rh_vel;
static HipVelEstimator_t s_lh_vel;
static ReactionController_t s_ctrl;
static DampingStreamPacket_t s_stream;
static float s_right_torque_velocity_dps = 0.0f;
static float s_left_torque_velocity_dps  = 0.0f;
static EsVelocityEnvelope_t s_es_right_velocity;
static EsVelocityEnvelope_t s_es_left_velocity;
static float s_es_interleg_angle_envelope_deg = 0.0f;
static float s_es_right_output_tau_nm = 0.0f;
static float s_es_left_output_tau_nm  = 0.0f;
static EsVelocityEnvelope_t s_ls_right_velocity;
static EsVelocityEnvelope_t s_ls_left_velocity;
static float s_ls_interleg_angle_envelope_deg = 0.0f;
static EsVelocityEnvelope_t s_trip_right_velocity;
static EsVelocityEnvelope_t s_trip_left_velocity;
static float s_trip_interleg_angle_envelope_deg = 0.0f;

static bool s_manualTripWaiting      = false;
static bool s_manualTripSeenAirborne = false;

static uint32_t s_now_tick = 0U;
static uint32_t s_previous_loop_tick = 0U;
static uint32_t s_previous_torque_tick = 0U;
static uint32_t s_usb_stream_tick = 0U;
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
static float _UpdateTorqueVelocity(float previous_velocity_dps,
                                   float input_velocity_dps,
                                   float dt_s);
static void _UpdateEsVelocityEnvelope(EsVelocityEnvelope_t *envelope,
                                      float input_velocity_dps,
                                      float dt_s);
static float _UpdateEsAngleEnvelope(float previous_angle_deg,
                                    float input_angle_deg,
                                    float dt_s);

static void _ResetReactionController(void);
static void _StartReaction(PerturbationType_t type, LegSide_t side);
static void _RunReactionController(float *rh_tau, float *lh_tau);
static void _StopReaction(void);

static float _GetFlexVelDps(LegSide_t side);
static float _ComputeDampingTorqueNm(LegSide_t side, float B_eff, float qdot_dps);
static bool  _PrimaryVelocityReversed(void);

static bool  _GetH10FootContact(LegSide_t side);
static void  _UpdateManualPerturbationButtons(void);
static void  _UpdateManualTripFootContact(void);
static void  _HandleManualButtonEvent(uint16_t button,
                                      XmBtnEvent_t event,
                                      PerturbationType_t type,
                                      LegSide_t side);
static void  _RecordButtonMarker(uint16_t button,
                                 uint16_t kind,
                                 bool accepted,
                                 PerturbationType_t type,
                                 LegSide_t side);
static void  _UpdateDampingStream(void);

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
    s_ctrl.peak_reached = false;
    s_ctrl.peak_tracking_velocity_dps = 0.0f;
    s_ctrl.previous_nonzero_velocity_dps = 0.0f;
    s_ctrl.previous_nonzero_velocity_valid = false;
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

    XM_SetUsbTotalDataStream(false);
    XM_SetUsbAutoStream(true);
    XM_SetUsbCustomMeta(USB_STREAM_ID, "[]");

    _ResetVelocityEstimator(&s_rh_vel);
    _ResetVelocityEstimator(&s_lh_vel);
    _ResetReactionController();

    s_right_torque_velocity_dps = 0.0f;
    s_left_torque_velocity_dps = 0.0f;
    s_es_right_velocity = (EsVelocityEnvelope_t){0.0f, 0.0f, 0.0f};
    s_es_left_velocity = (EsVelocityEnvelope_t){0.0f, 0.0f, 0.0f};
    s_es_interleg_angle_envelope_deg = 0.0f;
    s_trip_right_velocity = (EsVelocityEnvelope_t){0.0f, 0.0f, 0.0f};
    s_trip_left_velocity = (EsVelocityEnvelope_t){0.0f, 0.0f, 0.0f};
    s_trip_interleg_angle_envelope_deg = 0.0f;
    s_trip_right_velocity = (EsVelocityEnvelope_t){0.0f, 0.0f, 0.0f};
    s_trip_left_velocity = (EsVelocityEnvelope_t){0.0f, 0.0f, 0.0f};
    s_trip_interleg_angle_envelope_deg = 0.0f;
    s_ls_right_velocity = (EsVelocityEnvelope_t){0.0f, 0.0f, 0.0f};
    s_ls_left_velocity = (EsVelocityEnvelope_t){0.0f, 0.0f, 0.0f};
    s_ls_interleg_angle_envelope_deg = 0.0f;
    s_trip_right_velocity = (EsVelocityEnvelope_t){0.0f, 0.0f, 0.0f};
    s_trip_left_velocity = (EsVelocityEnvelope_t){0.0f, 0.0f, 0.0f};
    s_trip_interleg_angle_envelope_deg = 0.0f;
    s_es_right_output_tau_nm = 0.0f;
    s_es_left_output_tau_nm = 0.0f;
    s_ls_right_velocity = (EsVelocityEnvelope_t){0.0f, 0.0f, 0.0f};
    s_ls_left_velocity = (EsVelocityEnvelope_t){0.0f, 0.0f, 0.0f};
    s_ls_interleg_angle_envelope_deg = 0.0f;

    XM_SetControlMode(XM_CTRL_MONITOR);

    s_now_tick = XM_GetTick();
    s_previous_loop_tick = s_now_tick;
    s_previous_torque_tick = s_now_tick;
    s_usb_stream_tick = s_now_tick;
}

void User_Loop(void)
{
    if (!s_tsm) {
        return;
    }

    s_now_tick = XM_GetTick();
    g_daq_user_loop_count++;

    if ((s_now_tick - s_previous_loop_tick) > 2U) {
        g_daq_loop_tick_gap_count++;
    }
    s_previous_loop_tick = s_now_tick;

    if (!XM_IsCmConnected()) {
        XM_TSM_TransitionTo(s_tsm, XM_STATE_OFF);
    }

    XM_TSM_Run(s_tsm);
    _UpdateManualPerturbationButtons();
    _UpdateDampingStream();
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

    s_es_right_velocity = (EsVelocityEnvelope_t){0.0f, 0.0f, 0.0f};
    s_es_left_velocity = (EsVelocityEnvelope_t){0.0f, 0.0f, 0.0f};
    s_es_interleg_angle_envelope_deg = 0.0f;
    s_es_right_output_tau_nm = 0.0f;
    s_es_left_output_tau_nm = 0.0f;

    s_manualTripWaiting = false;
    s_manualTripSeenAirborne = false;
    s_previous_torque_tick = XM_GetTick();
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

    uint32_t torque_elapsed_ms = now - s_previous_torque_tick;
    s_previous_torque_tick = now;
    float torque_dt_s = (float)torque_elapsed_ms * 0.001f;

    s_right_torque_velocity_dps = _UpdateTorqueVelocity(
        s_right_torque_velocity_dps,
        g_right_hip_vel_dps,
        torque_dt_s
    );
    s_left_torque_velocity_dps = _UpdateTorqueVelocity(
        s_left_torque_velocity_dps,
        g_left_hip_vel_dps,
        torque_dt_s
    );

    if (s_ctrl.type == PERT_ES_SLIP) {
        _UpdateEsVelocityEnvelope(
            &s_es_right_velocity,
            g_right_hip_vel_dps,
            torque_dt_s
        );
        _UpdateEsVelocityEnvelope(
            &s_es_left_velocity,
            g_left_hip_vel_dps,
            torque_dt_s
        );
    } else if (s_ctrl.type == PERT_LS_SLIP) {
        _UpdateEsVelocityEnvelope(
            &s_ls_right_velocity,
            g_right_hip_vel_dps,
            torque_dt_s
        );
        _UpdateEsVelocityEnvelope(
            &s_ls_left_velocity,
            g_left_hip_vel_dps,
            torque_dt_s
        );
    } else if (s_ctrl.type == PERT_EARLY_SWING_TRIP &&
               s_ctrl.state == CTRL_ACTIVE) {
        _UpdateEsVelocityEnvelope(
            &s_trip_right_velocity,
            g_right_hip_vel_dps,
            torque_dt_s
        );
        _UpdateEsVelocityEnvelope(
            &s_trip_left_velocity,
            g_left_hip_vel_dps,
            torque_dt_s
        );
    }

    g_interleg_angle_deg =
        fabsf(g_right_hip_angle_deg - g_left_hip_angle_deg);

    g_angle_scale =
        1.0f + g_angle_gain_per_deg * g_interleg_angle_deg;

    if (s_ctrl.type == PERT_ES_SLIP) {
        s_es_interleg_angle_envelope_deg = _UpdateEsAngleEnvelope(
            s_es_interleg_angle_envelope_deg,
            g_interleg_angle_deg,
            torque_dt_s
        );
    } else if (s_ctrl.type == PERT_LS_SLIP) {
        s_ls_interleg_angle_envelope_deg = _UpdateEsAngleEnvelope(
            s_ls_interleg_angle_envelope_deg,
            g_interleg_angle_deg,
            torque_dt_s
        );
    } else if (s_ctrl.type == PERT_EARLY_SWING_TRIP &&
               s_ctrl.state == CTRL_ACTIVE) {
        s_trip_interleg_angle_envelope_deg = _UpdateEsAngleEnvelope(
            s_trip_interleg_angle_envelope_deg,
            g_interleg_angle_deg,
            torque_dt_s
        );
    }

    /* ---------------------------------------------------------------------- */
    /* B. Manual perturbation event input                                     */
    /* ---------------------------------------------------------------------- */

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

    s_es_right_velocity = (EsVelocityEnvelope_t){0.0f, 0.0f, 0.0f};
    s_es_left_velocity = (EsVelocityEnvelope_t){0.0f, 0.0f, 0.0f};
    s_es_interleg_angle_envelope_deg = 0.0f;

    s_ctrl.type  = type;
    s_ctrl.side  = side;

    g_active_event_type = (uint16_t)type;
    g_active_event_side = (uint16_t)side;

    s_ctrl.peak_reached = false;
    s_ctrl.peak_tracking_velocity_dps = 0.0f;
    s_ctrl.previous_nonzero_velocity_dps = 0.0f;
    s_ctrl.previous_nonzero_velocity_valid = false;
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
    if ((s_ctrl.type == PERT_ES_SLIP ||
         s_ctrl.type == PERT_EARLY_SWING_TRIP) &&
        s_ctrl.peak_reached) {
        if (s_ctrl.type == PERT_ES_SLIP) {
            rh_qdot = s_es_right_velocity.signed_velocity_dps;
            lh_qdot = s_es_left_velocity.signed_velocity_dps;
        } else {
            rh_qdot = s_trip_right_velocity.signed_velocity_dps;
            lh_qdot = s_trip_left_velocity.signed_velocity_dps;
        }
    } else if (s_ctrl.type == PERT_LS_SLIP && s_ctrl.peak_reached) {
        rh_qdot = s_ls_right_velocity.signed_velocity_dps;
        lh_qdot = s_ls_left_velocity.signed_velocity_dps;
    } else if (s_ctrl.type != PERT_ES_SLIP && s_ctrl.peak_reached) {
        rh_qdot = s_right_torque_velocity_dps;
        lh_qdot = s_left_torque_velocity_dps;
    }

    float rh_flex_vel = g_right_flex_vel_dps;
    float lh_flex_vel = g_left_flex_vel_dps;

    LegSide_t primary = s_ctrl.side;
    LegSide_t secondary =
        (primary == LEG_RIGHT) ? LEG_LEFT : LEG_RIGHT;

    float primary_flex_vel =
        (primary == LEG_RIGHT) ? rh_flex_vel : lh_flex_vel;

    float secondary_flex_vel =
        (secondary == LEG_RIGHT) ? rh_flex_vel : lh_flex_vel;

    if (s_ctrl.type == PERT_ES_SLIP) {
        primary_flex_vel =
            (primary == LEG_RIGHT) ?
            s_es_right_velocity.signed_velocity_dps :
            s_es_left_velocity.signed_velocity_dps;
    } else if (s_ctrl.type == PERT_LS_SLIP) {
        primary_flex_vel =
            (primary == LEG_RIGHT) ?
            s_ls_right_velocity.signed_velocity_dps :
            s_ls_left_velocity.signed_velocity_dps;
        secondary_flex_vel =
            (secondary == LEG_RIGHT) ?
            s_ls_right_velocity.signed_velocity_dps :
            s_ls_left_velocity.signed_velocity_dps;
    } else if (s_ctrl.type == PERT_EARLY_SWING_TRIP) {
        primary_flex_vel =
            (primary == LEG_RIGHT) ?
            s_trip_right_velocity.signed_velocity_dps :
            s_trip_left_velocity.signed_velocity_dps;
        secondary_flex_vel =
            (secondary == LEG_RIGHT) ?
            s_trip_right_velocity.signed_velocity_dps :
            s_trip_left_velocity.signed_velocity_dps;
    }

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
        B_secondary = g_k_es_stance *
                  (1.0f + g_angle_gain_per_deg *
                   s_es_interleg_angle_envelope_deg);

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
        B_secondary = g_k_ls_stance *
                  (1.0f + g_angle_gain_per_deg *
                   s_ls_interleg_angle_envelope_deg);

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
            g_k_trip_stance *
            (1.0f + g_angle_gain_per_deg *
             s_trip_interleg_angle_envelope_deg);

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
    /* ONLY STOP CONDITION: post-peak filtered velocity sign change           */
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

/* Detect the post-peak zero crossing of the torque-filtered velocity. */
static bool _PrimaryVelocityReversed(void)
{
    float raw_velocity = _GetFlexVelDps(s_ctrl.side);
    float filtered_velocity =
        (s_ctrl.side == LEG_RIGHT) ?
        ((s_ctrl.type == PERT_LS_SLIP) ?
         s_ls_right_velocity.signed_velocity_dps :
         (s_ctrl.type == PERT_EARLY_SWING_TRIP) ?
         s_trip_right_velocity.signed_velocity_dps :
         s_es_right_velocity.signed_velocity_dps) :
        ((s_ctrl.type == PERT_LS_SLIP) ?
         s_ls_left_velocity.signed_velocity_dps :
         (s_ctrl.type == PERT_EARLY_SWING_TRIP) ?
         s_trip_left_velocity.signed_velocity_dps :
         s_es_left_velocity.signed_velocity_dps);
    float harmful_velocity =
        (s_ctrl.primary_harmful_direction > 0) ?
        raw_velocity :
        -raw_velocity;

    if (!s_ctrl.peak_reached) {
        if (harmful_velocity > s_ctrl.peak_tracking_velocity_dps) {
            s_ctrl.peak_tracking_velocity_dps = harmful_velocity;
            return false;
        }

        if (s_ctrl.peak_tracking_velocity_dps > 0.0f &&
            harmful_velocity < s_ctrl.peak_tracking_velocity_dps) {
            s_ctrl.peak_reached = true;
            s_ctrl.previous_nonzero_velocity_valid = false;
        }

        return false;
    }

    if (fabsf(filtered_velocity) <= 0.001f) {
        return false;
    }

    if (s_ctrl.previous_nonzero_velocity_valid &&
        (filtered_velocity * s_ctrl.previous_nonzero_velocity_dps) < 0.0f) {
        g_last_stop_flex_vel_dps = filtered_velocity;
        return true;
    }

    s_ctrl.previous_nonzero_velocity_dps = filtered_velocity;
    s_ctrl.previous_nonzero_velocity_valid = true;
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
    s_ctrl.peak_reached = false;
    s_ctrl.peak_tracking_velocity_dps = 0.0f;
    s_ctrl.previous_nonzero_velocity_dps = 0.0f;
    s_ctrl.previous_nonzero_velocity_valid = false;

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

static float _UpdateTorqueVelocity(
    float previous_velocity_dps,
    float input_velocity_dps,
    float dt_s
)
{
    if (dt_s <= 0.0f) {
        return previous_velocity_dps;
    }

    float alpha = _LpfAlpha(TORQUE_VEL_LPF_HZ, dt_s);

    return previous_velocity_dps +
           alpha * (input_velocity_dps - previous_velocity_dps);
}

static void _UpdateEsVelocityEnvelope(
    EsVelocityEnvelope_t *envelope,
    float input_velocity_dps,
    float dt_s
)
{
    if (dt_s <= 0.0f) {
        return;
    }

    float alpha = _LpfAlpha(TORQUE_VEL_LPF_HZ, dt_s);
    float positive_input = (input_velocity_dps > 0.0f) ?
                           input_velocity_dps :
                           0.0f;
    float negative_input = (input_velocity_dps < 0.0f) ?
                           -input_velocity_dps :
                           0.0f;

    envelope->positive_envelope_dps +=
        alpha * (positive_input - envelope->positive_envelope_dps);
    envelope->negative_envelope_dps +=
        alpha * (negative_input - envelope->negative_envelope_dps);
    envelope->signed_velocity_dps =
        envelope->positive_envelope_dps -
        envelope->negative_envelope_dps;
}

static float _UpdateEsAngleEnvelope(
    float previous_angle_deg,
    float input_angle_deg,
    float dt_s
)
{
    if (dt_s <= 0.0f) {
        return previous_angle_deg;
    }

    float alpha = _LpfAlpha(TORQUE_VEL_LPF_HZ, dt_s);

    return previous_angle_deg +
           alpha * (input_angle_deg - previous_angle_deg);
}

/* ========================================================================== */
/* 14. XM PHYSICAL BUTTON MANUAL DETECTOR                                     */
/* ========================================================================== */

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
    g_manual_btn1_state_dbg = (uint16_t)XM_GetButtonState(XM_BTN_1);
    g_manual_btn2_state_dbg = (uint16_t)XM_GetButtonState(XM_BTN_2);
    g_manual_btn3_state_dbg = (uint16_t)XM_GetButtonState(XM_BTN_3);

    LegSide_t side =
        (g_manual_event_side == LEG_LEFT) ?
        LEG_LEFT :
        LEG_RIGHT;

    _HandleManualButtonEvent(
        1U,
        XM_GetButtonEvent(XM_BTN_1),
        PERT_ES_SLIP,
        side
    );
    _HandleManualButtonEvent(
        2U,
        XM_GetButtonEvent(XM_BTN_2),
        PERT_LS_SLIP,
        side
    );
    _HandleManualButtonEvent(
        3U,
        XM_GetButtonEvent(XM_BTN_3),
        PERT_EARLY_SWING_TRIP,
        side
    );
}

static void _HandleManualButtonEvent(
    uint16_t button,
    XmBtnEvent_t event,
    PerturbationType_t type,
    LegSide_t side
)
{
    if (event != XM_BTN_PRESSED) {
        return;
    }

    bool accepted =
        (XM.status.h10.h10Mode == XM_H10_MODE_ASSIST) &&
        (XM.command.control_mode == XM_CTRL_TORQUE) &&
        (s_ctrl.state == CTRL_IDLE);

    _RecordButtonMarker(
        button,
        BUTTON_MARKER_CLICK,
        accepted,
        type,
        side
    );

    if (!accepted) {
        XM_SendUsbDebugMessage(
            "[MANUAL] press logged but controller was not idle/active\r\n"
        );
        return;
    }

    g_manual_last_button = button;
    g_manual_trigger_count++;
    DampingAssist_Trigger(type, side);

    if (type == PERT_EARLY_SWING_TRIP) {
        s_manualTripWaiting = true;

        /* A click during swing immediately confirms the airborne phase. */
        s_manualTripSeenAirborne = !_GetH10FootContact(side);
        g_manual_trip_waiting_for_contact = 1U;
        g_manual_trip_seen_airborne =
            s_manualTripSeenAirborne ? 1U : 0U;

        XM_SendUsbDebugMessage(
            "[MANUAL] BTN3 press -> TRIP; waiting foot contact\r\n"
        );
    } else if (type == PERT_ES_SLIP) {
        XM_SendUsbDebugMessage("[MANUAL] BTN1 press -> ES SLIP\r\n");
    } else {
        XM_SendUsbDebugMessage("[MANUAL] BTN2 press -> LS SLIP\r\n");
    }
}

static void _RecordButtonMarker(
    uint16_t button,
    uint16_t kind,
    bool accepted,
    PerturbationType_t type,
    LegSide_t side
)
{
    g_button_marker_code = button;
    g_button_marker_kind = kind;
    g_button_marker_tick_ms = s_now_tick;
    g_button_marker_accepted = accepted ? 1U : 0U;
    g_button_marker_event_type = (uint16_t)type;
    g_button_marker_event_side = (uint16_t)side;
    g_button_marker_sequence++;
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

static void _UpdateDampingStream(void)
{
    /* Keep auto-stream policy enabled; packet transmission below is never
     * gated by H10 mode or by XM_IsUsbStreamingActive(). */
    if (!XM_IsUsbStreamingActive()) {
        XM_SetUsbAutoStream(true);
    }

    if ((s_now_tick - s_usb_stream_tick) < USB_STREAM_PERIOD_MS) {
        return;
    }
    s_usb_stream_tick = s_now_tick;

#define ASSIGN_STREAM_FIELD(name, source) s_stream.name = (float)(source);
    DAMPING_STREAM_CHANNELS(ASSIGN_STREAM_FIELD, ASSIGN_STREAM_FIELD)
#undef ASSIGN_STREAM_FIELD

    if (!XM_SendUsbDataWithId(
            &s_stream,
            sizeof(s_stream),
            USB_STREAM_ID
        )) {
        g_daq_usb_tx_fail_count++;
    }

    g_daq_robot_frame_sequence++;
}

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
