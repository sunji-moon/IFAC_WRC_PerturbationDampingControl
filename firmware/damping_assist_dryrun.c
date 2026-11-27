/*
 * Dry-run build variant for damping_assist.c.
 *
 * The SJ controller still calculates and logs the requested torque, but the
 * H10 torque API calls are replaced with no-ops. The command fields exposed to
 * Live Expressions and the HND logger are forced to 0 Nm after each packet is
 * prepared, so the trial measures controller behavior without motor torque.
 *
 * Build this file instead of damping_assist.c. Do not build
 * both files together: both define the XM User_Setup/User_Loop entry points.
 */

#include "xm_api.h"

#include <stddef.h>
#include <stdint.h>

extern volatile float g_right_tau_cmd_nm;
extern volatile float g_left_tau_cmd_nm;

static bool _DryRun_SendDampingPacket(
    void *data,
    size_t size,
    uint8_t stream_id
)
{
    /* DampingStreamPacket_t contains 70 float fields in the SJ implementation.
     * left_tau_cmd_nm and right_tau_cmd_nm are fields 50 and 51. */
    if (size == (70U * sizeof(float))) {
        float *fields = (float *)data;
        fields[50] = 0.0f;
        fields[51] = 0.0f;
    }

    g_right_tau_cmd_nm = 0.0f;
    g_left_tau_cmd_nm = 0.0f;

    return XM_SendUsbDataWithId(data, size, stream_id);
}

static void _DryRun_SetAssistTorqueRH(float ignored_value)
{
    (void)ignored_value;
    XM_SetAssistTorqueRH(0.0f);
}

static void _DryRun_SetAssistTorqueLH(float ignored_value)
{
    (void)ignored_value;
    XM_SetAssistTorqueLH(0.0f);
}

#define XM_SetAssistTorqueRH(value) \
    _DryRun_SetAssistTorqueRH((value))
#define XM_SetAssistTorqueLH(value) \
    _DryRun_SetAssistTorqueLH((value))
#define XM_SendUsbDataWithId(data, size, stream_id) \
    _DryRun_SendDampingPacket((data), (size), (stream_id))
#define User_Setup DryRun_SJ_User_Setup
#define User_Loop DryRun_SJ_User_Loop

#include "damping_assist.c"

#undef User_Loop
#undef User_Setup
#undef XM_SendUsbDataWithId
#undef XM_SetAssistTorqueLH
#undef XM_SetAssistTorqueRH

void User_Setup(void)
{
    DryRun_SJ_User_Setup();
}

void User_Loop(void)
{
    DryRun_SJ_User_Loop();
}
