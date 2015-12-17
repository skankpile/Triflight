/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>
#include <math.h>

#include <platform.h>

#include "build_config.h"

#include "common/axis.h"
#include "common/maths.h"
#include "common/filter.h"

#include "drivers/sensor.h"
#include "drivers/accgyro.h"
#include "drivers/gyro_sync.h"

#include "sensors/sensors.h"
#include "sensors/gyro.h"
#include "sensors/acceleration.h"

#include "rx/rx.h"

#include "io/rc_controls.h"
#include "io/gps.h"

#include "flight/pid.h"
#include "flight/imu.h"
#include "flight/navigation.h"
#include "flight/gtune.h"

#include "config/runtime_config.h"

#include "debug.h"

#include "mw.h"

//! Integrator is disabled when rate error exceeds this limit
#define LUXFLOAT_INTEGRATOR_DISABLE_LIMIT_DPS (30.0f)
#define GYRO_RATE_HZ 1000U  // Gyro refresh rate 1khz

//typedef enum {
//    GYRO_BASED = 0,
//    ATTITUDE_BASED,
//} yaw_mode_t;

extern uint16_t cycleTime;
extern uint8_t motorCount;
extern float dT;

int16_t axisPID[3];

#ifdef BLACKBOX
int32_t axisPID_P[3], axisPID_I[3], axisPID_D[3];
#endif

// PIDweight is a scale factor for PIDs which is derived from the throttle and TPA setting, and 100 = 100% scale means no PID reduction
uint8_t dynP8[3], dynI8[3], dynD8[3], PIDweight[3], Iweigth[3] = {0};

static int32_t errorGyroI[3] = { 0, 0, 0 };
static float errorGyroIf[3] = { 0.0f, 0.0f, 0.0f };
static int32_t errorAngleI[2] = { 0, 0 };
static float errorAngleIf[2] = { 0.0f, 0.0f };

//static yaw_mode_t gYawMode = GYRO_BASED;

static void pidRewrite(pidProfile_t *pidProfile, controlRateConfig_t *controlRateConfig,
        uint16_t max_angle_inclination, rollAndPitchTrims_t *angleTrim, rxConfig_t *rxConfig);
static int16_t pidLuxFloatHeading(flight_dynamics_index_t axis, uint8_t rate, pidProfile_t *pidProfile);

typedef void (*pidControllerFuncPtr)(pidProfile_t *pidProfile, controlRateConfig_t *controlRateConfig,
        uint16_t max_angle_inclination, rollAndPitchTrims_t *angleTrim, rxConfig_t *rxConfig);            // pid controller function prototype

pidControllerFuncPtr pid_controller = pidRewrite; // which pid controller are we using, defaultMultiWii

void pidResetErrorAngle(void)
{
    errorAngleI[ROLL] = 0;
    errorAngleI[PITCH] = 0;

    errorAngleIf[ROLL] = 0.0f;
    errorAngleIf[PITCH] = 0.0f;
}

void pidResetErrorGyro(void)
{
    errorGyroI[ROLL] = 0;
    errorGyroI[PITCH] = 0;
    errorGyroI[YAW] = 0;

    errorGyroIf[ROLL] = 0.0f;
    errorGyroIf[PITCH] = 0.0f;
    errorGyroIf[YAW] = 0.0f;
}

const angle_index_t rcAliasToAngleIndexMap[] = { AI_ROLL, AI_PITCH };

static filterStatePt1_t DTermState[3];
static filterStatePt1_t DTermStateHeading[3];
static float headingSetpoint[3];

//float getYawSetpoint(uint8_t rate)
//{
//    static bool firstTime = true;
//    float setpoint;
//
//    if (firstTime)
//    {
//        headingSetpoint = attitude.values.yaw;
//        firstTime = false;
//    }
//    if ((attitude.values.roll > 600) || (attitude.values.pitch > 600))
//    {
//        // Gyro based
//        gYawMode = GYRO_BASED;
//        setpoint = (float)((rate + 10) * rcCommand[YAW]) / 50.0f;
//        headingSetpoint = attitude.values.yaw;
//    }
//    else
//    {
//        // Attitude based
//        gYawMode = ATTITUDE_BASED;
//        headingSetpoint += rcCommand[YAW] * rate / 10 * dT;
//
//        int16_t diff = headingSetpoint - attitude.values.yaw;
//
//
//        if (diff >= 1800)
//        {
//            headingSetpoint -= 3600;
//        }
//        else if (diff <= -1800)
//        {
//            headingSetpoint += 3600;
//        }
//        setpoint = headingSetpoint / 10.0f;
//
//    }
//
//    return setpoint;
//}
//
//float getAxisFeedback(flight_dynamics_index_t axis)
//{
//    float feedback;
//
//    switch (axis)
//    {
//    case FD_YAW:
//        if (gYawMode == ATTITUDE_BASED)
//        {
//            feedback = attitude.values.yaw / 10.0f;
//            break;
//        }
//        // intentional fallthrough to default
//    default:
//        feedback = gyroADC[axis] * gyro.scale;
//        break;
//    }
//
//    return feedback;
//}

//float correctErrorSign(flight_dynamics_index_t axis, float error)
//{
//    switch (axis)
//    {
//    case FD_YAW:
//        if (gYawMode == ATTITUDE_BASED)
//        {
//            error = -15.0f * error;
//            break;
//        }
//        // intentional fallthrough to default
//    default:
//        break;
//    }
//
//    return error;
//}

float getHeadingSetPoint(uint8_t axis)
{
    return headingSetpoint[axis];
}

float increaseHeadingSetPoint(flight_dynamics_index_t axis, float amount, float axisHeading)
{
    headingSetpoint[axis] -= amount;

    if (headingSetpoint[axis] < 0.0f)
    {
        headingSetpoint[axis] += 360.0f;
    }
    else if (headingSetpoint[axis] > 360.0f)
    {
        headingSetpoint[axis] -= 360.0f;
    }

    float angleDiff = axisHeading - headingSetpoint[axis];

    if (angleDiff >= 180.0f)
    {
        angleDiff -= 360.0f;
    }
    else if (angleDiff <= -180.0f)
    {
        angleDiff += 360.0f;
    }

    return angleDiff;
}

float getHeadingError(flight_dynamics_index_t axis, uint8_t rate)
{
    float headingError;
    float direction = 1.0f;
    int16_t axisHeading;

//    if (!IS_RC_MODE_ACTIVE(BOXTAILTUNE))
//    {
//        return 0.0f;
//    }

    switch (axis)
    {
    case FD_ROLL:
        return 0.0f;
        axisHeading = attitude.values.roll;
        direction = -1.0f;
        break;
    case FD_PITCH:
        return 0.0f;
        axisHeading = attitude.values.pitch;
        direction = -1.0f;
        break;
    case FD_YAW:
        axisHeading = attitude.values.yaw;
        break;
    default:
        // Wrong parameter value
        axisHeading = 0;
        break;
    }

    if ((ABS(attitude.values.roll) < 600) && (ABS(attitude.values.pitch) < 600))
    {
//        headingSetpoint[axis] -= (float)(rcCommand[YAW] * rate) / 15.0f * dT;
        float yawStickDeflection = (float)ABS(rcCommand[YAW]) / 500.0f;
        float angleDiff;
        debug[1] = yawStickDeflection * 100;
        angleDiff = increaseHeadingSetPoint(axis, (float)(rcCommand[YAW] * (rate + 10)) / 25.0f * dT, (float)axisHeading / 10.0f);
        angleDiff = increaseHeadingSetPoint(axis, -0.1f * angleDiff * yawStickDeflection, (float)axisHeading / 10.0f);

        headingError = direction * angleDiff * 6.0f;

        //float headingErrorGain = 1.0f - ((float)ABS(rcCommand[YAW]) / 500.0f);
        debug[3] = headingError;

    }
    else
    {
        // Not active when inverted
        headingSetpoint[axis] = axisHeading / 10.0f;
        headingError = 0.0f;
    }
//    switch (axis)
//    {
//    case FD_YAW:
//        // Not active when inverted
//        if ((attitude.values.roll < 600) || (attitude.values.pitch < 600))
//        {
//            if (!isRcAxisWithinDeadband(axis))
//            {
//                headingSetpoint[axis] = attitude.values.yaw;
//            }
//
//            int16_t diff = attitude.values.yaw - headingSetpoint;
//            if (diff >= 1800)
//            {
//                diff -= 3600;
//            }
//            else if (diff <= -1800)
//            {
//                diff += 3600;
//            }
//
//            headingError = diff;
//
//            debug[0] = headingError;
//            break;
//        }
//        else
//        {
//            headingSetpoint = attitude.values.yaw;
//        }
//        // intentional fallthrough to default
//    default:
//        break;
//    }
//    switch (axis)
//    {
//    case FD_ROLL:
//        debug[0] = headingError;
//        break;
//    case FD_PITCH:
//        debug[1] = headingError;
//        break;
//    case FD_YAW:
//        debug[2] = headingError;
//        break;
//    default:
//        // Wrong parameter value
//        axisHeading = 0;
//        break;
//    }
//    debug[2] = headingError;
    return headingError;
}

static void pidLuxFloat(pidProfile_t *pidProfile, controlRateConfig_t *controlRateConfig,
        uint16_t max_angle_inclination, rollAndPitchTrims_t *angleTrim, rxConfig_t *rxConfig)
{
    float rateError, errorAngle, setpoint, feedback;
    float ITerm,PTerm,DTerm;
    int32_t stickPosAil, stickPosEle, mostDeflectedPos;
    static float lastError[3];
    float delta;
    int axis;
    float horizonLevelStrength = 1;

    if (FLIGHT_MODE(HORIZON_MODE)) {

        // Figure out the raw stick positions
        stickPosAil = getRcStickDeflection(FD_ROLL, rxConfig->midrc);
        stickPosEle = getRcStickDeflection(FD_PITCH, rxConfig->midrc);

        if(ABS(stickPosAil) > ABS(stickPosEle)){
            mostDeflectedPos = ABS(stickPosAil);
        }
        else {
            mostDeflectedPos = ABS(stickPosEle);
        }

        // Progressively turn off the horizon self level strength as the stick is banged over
        horizonLevelStrength = (float)(500 - mostDeflectedPos) / 500;  // 1 at centre stick, 0 = max stick deflection
        if(pidProfile->H_sensitivity == 0){
            horizonLevelStrength = 0;
        } else {
            horizonLevelStrength = constrainf(((horizonLevelStrength - 1) * (100 / pidProfile->H_sensitivity)) + 1, 0, 1);
        }
    }

    // ----------PID controller----------
    for (axis = 0; axis < 3; axis++) {
        // -----Get the desired angle rate depending on flight mode
        uint8_t rate = controlRateConfig->rates[axis];
        if (axis == FD_YAW) {
            // YAW is always gyro-controlled (MAG correction is applied to rcCommand) 100dps to 1100dps max yaw rate
            setpoint = (float)((rate + 10) * rcCommand[YAW]) / 50.0f;
         } else {
            // calculate error and limit the angle to the max inclination
#ifdef GPS
            errorAngle = (constrainf(((float)rcCommand[axis] * ((float)max_angle_inclination / 500.0f)) + GPS_angle[axis], -((int) max_angle_inclination),
                    +max_angle_inclination) - attitude.raw[axis] + angleTrim->raw[axis]) / 10.0f;
#else
            errorAngle = (constrainf((float)rcCommand[axis] * ((float)max_angle_inclination / 500.0f), -((int) max_angle_inclination),
                    +max_angle_inclination) - attitude.raw[axis] + angleTrim->raw[axis]) / 10.0f;
#endif

            if (FLIGHT_MODE(ANGLE_MODE)) {
                // it's the ANGLE mode - control is angle based, so control loop is needed
                setpoint = errorAngle * pidProfile->A_level;
            } else {
                //control is GYRO based (ACRO and HORIZON - direct sticks control is applied to rate PID
                setpoint = (float)((rate + 20) * rcCommand[axis]) / 50.0f; // 200dps to 1200dps max roll/pitch rate
                if (FLIGHT_MODE(HORIZON_MODE)) {
                    // mix up angle error to desired AngleRate to add a little auto-level feel
                    setpoint += errorAngle * pidProfile->H_level * horizonLevelStrength;
                }
            }
        }

        feedback = gyroADC[axis] * gyro.scale; // gyro output scaled to dps

        // --------low-level gyro-based PID. ----------
        // Used in stand-alone mode for ACRO, controlled by higher level regulators in other modes
        // -----calculate scaled error.AngleRates
        // multiplication of rcCommand corresponds to changing the sticks scaling here
        rateError = setpoint - feedback;

        // -----calculate P component
        PTerm = rateError * pidProfile->P_f[axis] * PIDweight[axis] / 100;

        // -----calculate I component.
        float iferrornormal = (rateError) * dT * pidProfile->I_f[axis] * 10 * Iweigth[axis] / 100;
        errorGyroIf[axis] = constrainf(errorGyroIf[axis] + iferrornormal, -250.0f, 250.0f);
        // limit maximum integrator value to prevent WindUp - accumulating extreme values when system is saturated.
        // I coefficient (I8) moved before integration to make limiting independent from PID settings
        ITerm = errorGyroIf[axis];

        //-----calculate D-term
        delta = rateError - lastError[axis];
        lastError[axis] = rateError;

        // Correct difference by cycle time. Cycle time is jittery (can be different 2 times), so calculated difference
        // would be scaled by different dt each time. Division by dT fixes that.
        delta *= (1.0f / dT);

        // Dterm low pass
        if (pidProfile->dterm_cut_hz) {
            delta = filterApplyPt1(delta, &DTermState[axis], pidProfile->dterm_cut_hz, dT);
        }

        DTerm = constrainf(delta * pidProfile->D_f[axis] * PIDweight[axis] / 100, -300.0f, 300.0f);

        // -----calculate total PID output
        axisPID[axis] = constrain(lrintf(PTerm + ITerm + DTerm + pidLuxFloatHeading(axis, rate, pidProfile)), -1000, 1000);

#ifdef GTUNE
        if (FLIGHT_MODE(GTUNE_MODE) && ARMING_FLAG(ARMED)) {
            calculate_Gtune(axis);
        }
#endif

#ifdef BLACKBOX
        if (axis != FD_PITCH)
        {
            axisPID_P[axis] = PTerm;
            axisPID_I[axis] = ITerm;
            axisPID_D[axis] = DTerm;
        }
#endif
    }
}

static int16_t pidLuxFloatHeading(flight_dynamics_index_t axis, uint8_t rate, pidProfile_t *pidProfile)
{
    float headingError;
    float ITerm,PTerm,DTerm;
    static float lastError[3];
    static const float P_f[3] = {
            [FD_ROLL]  = 0.0f,
            [FD_PITCH] = 0.0f,
            [FD_YAW]   = 2.0f};
    static const float I_f[3] = {
            [FD_ROLL]  = 0.0f,
            [FD_PITCH] = 0.0f,
            [FD_YAW]   = 0.04f};
    static const float D_f[3] = {
            [FD_ROLL]  = 0.0f,
            [FD_PITCH] = 0.0f,
            [FD_YAW]   = 0.30f};
    float delta;

    if (axis != FD_YAW)
    {
        return 0;
    }

    headingError = getHeadingError(axis, rate);

    // -----calculate P component
    PTerm = headingError * P_f[axis] * PIDweight[axis] / 100;

    // -----calculate I component.
    float iferrornormal = (headingError) * dT * I_f[axis] * 10 * Iweigth[axis] / 100;
    errorGyroIf[axis] = constrainf(errorGyroIf[axis] + iferrornormal, -250.0f, 250.0f);
    // limit maximum integrator value to prevent WindUp - accumulating extreme values when system is saturated.
    // I coefficient (I8) moved before integration to make limiting independent from PID settings
    ITerm = errorGyroIf[axis];

    //-----calculate D-term
    delta = headingError - lastError[axis];
    lastError[axis] = headingError;

    // Correct difference by cycle time. Cycle time is jittery (can be different 2 times), so calculated difference
    // would be scaled by different dt each time. Division by dT fixes that.
    delta *= (1.0f / dT);

    // Dterm low pass
    if (pidProfile->dterm_cut_hz) {
        delta = filterApplyPt1(delta, &DTermStateHeading[axis], pidProfile->dterm_cut_hz, dT);
    }
    DTerm = constrainf(delta * D_f[axis] * PIDweight[axis] / 100, -300.0f, 300.0f);

#ifdef BLACKBOX
    axisPID_P[FD_PITCH] = PTerm;
    axisPID_I[FD_PITCH] = ITerm;
    axisPID_D[FD_PITCH] = DTerm;
#endif
    debug[0] = getHeadingSetPoint(FD_YAW);
    // -----calculate total PID output
    return constrain(lrintf(PTerm + ITerm + DTerm), -1000, 1000);
}


static void pidRewrite(pidProfile_t *pidProfile, controlRateConfig_t *controlRateConfig, uint16_t max_angle_inclination,
        rollAndPitchTrims_t *angleTrim, rxConfig_t *rxConfig)
{
    UNUSED(rxConfig);

    int32_t errorAngle;
    int axis;
    int32_t delta;
    int32_t PTerm, ITerm, DTerm;
    static int32_t lastError[3] = { 0, 0, 0 };
    int32_t AngleRateTmp, RateError;

    int8_t horizonLevelStrength = 100;
    int32_t stickPosAil, stickPosEle, mostDeflectedPos;

    if (FLIGHT_MODE(HORIZON_MODE)) {

        // Figure out the raw stick positions
        stickPosAil = getRcStickDeflection(FD_ROLL, rxConfig->midrc);
        stickPosEle = getRcStickDeflection(FD_PITCH, rxConfig->midrc);

        if(ABS(stickPosAil) > ABS(stickPosEle)){
            mostDeflectedPos = ABS(stickPosAil);
        }
        else {
            mostDeflectedPos = ABS(stickPosEle);
        }

        // Progressively turn off the horizon self level strength as the stick is banged over
        horizonLevelStrength = (500 - mostDeflectedPos) / 5;  // 100 at centre stick, 0 = max stick deflection

        // Using Level D as a Sensitivity for Horizon. 0 more level to 255 more rate. Default value of 100 seems to work fine.
        // For more rate mode increase D and slower flips and rolls will be possible
       	horizonLevelStrength = constrain((10 * (horizonLevelStrength - 100) * (10 * pidProfile->D8[PIDLEVEL] / 80) / 100) + 100, 0, 100);
    }

    // ----------PID controller----------
    for (axis = 0; axis < 3; axis++) {
        uint8_t rate = controlRateConfig->rates[axis];

        // -----Get the desired angle rate depending on flight mode
        if (axis == FD_YAW) { // YAW is always gyro-controlled (MAG correction is applied to rcCommand)
            AngleRateTmp = (((int32_t)(rate + 27) * rcCommand[YAW]) >> 5);
        } else {
            // calculate error and limit the angle to max configured inclination
#ifdef GPS
            errorAngle = constrain(2 * rcCommand[axis] + GPS_angle[axis], -((int) max_angle_inclination),
                    +max_angle_inclination) - attitude.raw[axis] + angleTrim->raw[axis]; // 16 bits is ok here
#else
            errorAngle = constrain(2 * rcCommand[axis], -((int) max_angle_inclination),
                    +max_angle_inclination) - attitude.raw[axis] + angleTrim->raw[axis]; // 16 bits is ok here
#endif

            if (!FLIGHT_MODE(ANGLE_MODE)) { //control is GYRO based (ACRO and HORIZON - direct sticks control is applied to rate PID
                AngleRateTmp = ((int32_t)(rate + 27) * rcCommand[axis]) >> 4;
                if (FLIGHT_MODE(HORIZON_MODE)) {
                    // mix up angle error to desired AngleRateTmp to add a little auto-level feel. horizonLevelStrength is scaled to the stick input
                	AngleRateTmp += (errorAngle * pidProfile->I8[PIDLEVEL] * horizonLevelStrength / 100) >> 4;
                }
            } else { // it's the ANGLE mode - control is angle based, so control loop is needed
                AngleRateTmp = (errorAngle * pidProfile->P8[PIDLEVEL]) >> 4;
            }
        }

        // --------low-level gyro-based PID. ----------
        // Used in stand-alone mode for ACRO, controlled by higher level regulators in other modes
        // -----calculate scaled error.AngleRates
        // multiplication of rcCommand corresponds to changing the sticks scaling here
        RateError = AngleRateTmp - (gyroADC[axis] / 4);

        // -----calculate P component
        PTerm = (RateError * pidProfile->P8[axis] * PIDweight[axis] / 100) >> 7;

        // -----calculate I component
        // there should be no division before accumulating the error to integrator, because the precision would be reduced.
        // Precision is critical, as I prevents from long-time drift. Thus, 32 bits integrator is used.
        // Time correction (to avoid different I scaling for different builds based on average cycle time)
        // is normalized to cycle time = 2048.
        errorGyroI[axis] = errorGyroI[axis] + ((RateError * (uint16_t)targetLooptime) >> 11) * pidProfile->I8[axis];

        // limit maximum integrator value to prevent WindUp - accumulating extreme values when system is saturated.
        // I coefficient (I8) moved before integration to make limiting independent from PID settings
        errorGyroI[axis] = constrain(errorGyroI[axis], (int32_t) - GYRO_I_MAX << 13, (int32_t) + GYRO_I_MAX << 13);
        ITerm = errorGyroI[axis] >> 13;

        //-----calculate D-term
        delta = RateError - lastError[axis]; // 16 bits is ok here, the dif between 2 consecutive gyro reads is limited to 800
        lastError[axis] = RateError;

        // Correct difference by cycle time. Cycle time is jittery (can be different 2 times), so calculated difference
        // would be scaled by different dt each time. Division by dT fixes that.
        delta = (delta * ((uint16_t) 0xFFFF / ((uint16_t)targetLooptime >> 4))) >> 6;

        // Dterm delta low pass
        if (pidProfile->dterm_cut_hz) {
            delta = filterApplyPt1(delta, &DTermState[axis], pidProfile->dterm_cut_hz, dT);
        }

        DTerm = (delta * 3 * pidProfile->D8[axis] * PIDweight[axis] / 100) >> 8; // Multiplied by 3 to match old scaling

        // -----calculate total PID output
        axisPID[axis] = PTerm + ITerm + DTerm;

#ifdef GTUNE
        if (FLIGHT_MODE(GTUNE_MODE) && ARMING_FLAG(ARMED)) {
             calculate_Gtune(axis);
        }
#endif

#ifdef BLACKBOX
        axisPID_P[axis] = PTerm;
        axisPID_I[axis] = ITerm;
        axisPID_D[axis] = DTerm;
#endif
    }
}

void pidSetController(pidControllerType_e type)
{
    switch (type) {
        default:
        case PID_CONTROLLER_REWRITE:
            pid_controller = pidRewrite;
            break;
        case PID_CONTROLLER_LUX_FLOAT:
            pid_controller = pidLuxFloat;
    }
}

