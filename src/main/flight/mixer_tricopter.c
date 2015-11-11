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
#include <math.h>
#include "platform.h"
#include "debug.h"

#include "common/axis.h"
#include "common/maths.h"

#include "drivers/system.h"
#include "drivers/pwm_output.h"
#include "drivers/pwm_mapping.h"
#include "drivers/sensor.h"
#include "drivers/accgyro.h"
#include "drivers/system.h"

#include "rx/rx.h"

#include "io/gimbal.h"
#include "io/escservo.h"
#include "io/rc_controls.h"

#include "sensors/sensors.h"
#include "sensors/acceleration.h"
#include "sensors/gyro.h"

#include "flight/mixer.h"
#include "flight/failsafe.h"
#include "flight/pid.h"
#include "flight/imu.h"
#include "flight/lowpass.h"

#include "config/runtime_config.h"
#include "config/config.h"


#ifdef USE_SERVOS
#define TRI_TAIL_SERVO_ANGLE_MID (900)
#define TRI_YAW_FORCE_CURVE_SIZE (100)
#define TRI_TAIL_SERVO_MAX_ANGLE (500)

#define SERVO_CALIB_NUM_OF_MEAS (5)
#define IsDelayElapsed(timestamp_us, delay_us) ((uint32_t)(micros() - timestamp_us) >= delay_us)

typedef enum {
    IDLE = 0,
    INIT,
    LISTEN_SILENCE,
    SERVO_TO_START,
    DETECT_START,
    DETECT_END,
    CHECK_RESULTS,
    DONE
} servoCalibState_e;

typedef struct servoCalibration_s {
    uint8_t active;
    uint8_t numOfMeasurements;
    uint8_t counter;
    uint16_t servoPosition;
    int16_t maxGyroSumIdle;
    servoCalibState_e state;
    uint16_t sumMeasurements;
    float lowestMeasurement;
    float highestMeasurement;
    uint32_t timestamp_us;
    bool firstTime;
    uint32_t baseTime;
    uint8_t counterLimit;
    uint16_t gyroPeakSum;
} servoCalibration_t;

#endif

#ifdef USE_SERVOS
extern float dT;

static servoCalibration_t servoCalib = {.state = IDLE};
static int16_t tailServoMaxYawForce = 0;
static float tailServoThrustFactor = 0;
static int16_t tailServoMaxAngle = 0;
static int16_t tailServoSpeed = 0;
static float virtualServoAngle = TRI_TAIL_SERVO_ANGLE_MID / 10.0f;
static int16_t yawForceCurve[TRI_YAW_FORCE_CURVE_SIZE];
static int16_t tailMotorPitchZeroAngle;
static int16_t tailMotorAccelerationDelay_ms = 30;
static int16_t tailMotorDecelerationDelay_ms = 100;
static int16_t tailMotorAccelerationDelay_angle;
static int16_t tailMotorDecelerationDelay_angle;

static servoParam_t * gpTailServoConf;
static int16_t *gpTailServo;
static mixerConfig_t *gpMixerConfig;

static uint16_t getServoValueAtAngle(servoParam_t * servoConf, uint16_t angle);
static float getPitchCorrectionAtTailAngle(float angle);
static uint16_t getAngleFromYawCurveAtForce(int16_t force);
static uint16_t getServoAngle(servoParam_t * servoConf, uint16_t servoValue);
static uint16_t getPitchCorrectionMaxPhaseShift(int16_t servoAngle,
        int16_t servoSetpointAngle,
        int16_t motorAccelerationDelayAngle,
        int16_t motorDecelerationDelayAngle,
        int16_t motorDirectionChangeAngle);
static int16_t getGyroSum();
static void virtualServoStep(float dT, servoParam_t *servoConf, uint16_t servoValue);
static void triServoCalibrationStep();

#endif

void triInitMixer(servoParam_t *pTailServoConfig,
        int16_t *pTailServo,
        mixerConfig_t *mixerConfig)
{
    gpTailServoConf = pTailServoConfig;
    gpTailServo = pTailServo;

    tailServoThrustFactor = mixerConfig->tri_tail_motor_thrustfactor / 10.0f;
    tailServoMaxAngle = mixerConfig->tri_servo_angle_at_max;
    tailServoSpeed = mixerConfig->tri_tail_servo_speed;
    gpMixerConfig = mixerConfig;
    // DERIVATE(1/(sin(x)-cos(x)/tailServoThrustFactor)) = 0
    // Multiplied by 10 to get decidegrees
    tailMotorPitchZeroAngle = 10.0f * 2.0f * (atanf(((sqrtf(tailServoThrustFactor * tailServoThrustFactor + 1) + 1) / tailServoThrustFactor)));

    tailMotorAccelerationDelay_angle = 10.0f * (tailMotorAccelerationDelay_ms / 1000.0f) * tailServoSpeed;
    tailMotorDecelerationDelay_angle = 10.0f * (tailMotorDecelerationDelay_ms / 1000.0f) * tailServoSpeed;

    const int16_t minAngle = TRI_TAIL_SERVO_ANGLE_MID - tailServoMaxAngle;
    const int16_t maxAngle = TRI_TAIL_SERVO_ANGLE_MID + tailServoMaxAngle;
    int16_t maxNegForce = 0;
    int16_t maxPosForce = 0;

    int16_t angle = TRI_TAIL_SERVO_ANGLE_MID - TRI_TAIL_SERVO_MAX_ANGLE;
    for (int32_t i = 0; i < TRI_YAW_FORCE_CURVE_SIZE; i++)
    {
        const float angleRad = DEGREES_TO_RADIANS(angle / 10.0f);
        yawForceCurve[i] = 1000.0f * (-tailServoThrustFactor * cosf(angleRad) - sinf(angleRad)) * getPitchCorrectionAtTailAngle(angleRad);
        // Only calculate the top forces in the configured angle range
        if ((angle >= minAngle) && (angle <= maxAngle))
        {
            maxNegForce = MIN(yawForceCurve[i], maxNegForce);
            maxPosForce = MAX(yawForceCurve[i], maxPosForce);
        }
        angle += 10;
    }

    tailServoMaxYawForce = MIN(ABS(maxNegForce), ABS(maxPosForce));
}

float triGetVirtualServoAngle()
{
    return virtualServoAngle;
}

uint16_t triGetLinearServoValue(servoParam_t *servoConf, uint16_t servoValue)
{
    const int16_t servoMid = servoConf->middle;
    // First find the yaw force at given servo value from a linear curve
    const int16_t servoRange = (servoValue < servoMid) ? servoMid - servoConf->min : servoConf->max - servoMid;
    const int16_t linearYawForceAtValue = (int32_t)(tailServoMaxYawForce) * (servoValue - servoMid) / servoRange;
    const int16_t correctedAngle = getAngleFromYawCurveAtForce(linearYawForceAtValue);
    return getServoValueAtAngle(servoConf, correctedAngle);
}

static int16_t getGyroSum()
{
    return ABS(gyroADC[X]) + ABS(gyroADC[Y]) + ABS(gyroADC[Z]);
}

void triServoMixer()
{
    if (ARMING_FLAG(ARMED))
    {
        *gpTailServo = triGetLinearServoValue(gpTailServoConf, *gpTailServo);
    }
    else
    {
        triServoCalibrationStep();
        if (servoCalib.active)
        {
            *gpTailServo = servoCalib.servoPosition;
        }
    }

    virtualServoStep(dT, gpTailServoConf, *gpTailServo);
}

int16_t triGetMotorCorrection(uint8_t motorIndex)
{
    uint16_t correction = 0;
    if (motorIndex == 0)
    {
        // Adjust tail motor speed based on servo angle. Check how much to adjust speed from pitch force curve based on servo angle.
        // Take motor speed up lag into account by shifting the phase of the curve
        // Not taking into account the motor braking lag (yet)
        const int16_t servoAngle = triGetVirtualServoAngle() * 10.0f;
        const int16_t servoSetpointAngle = getServoAngle(gpTailServoConf, *gpTailServo);

        uint16_t maxPhaseShift = getPitchCorrectionMaxPhaseShift(servoAngle, servoSetpointAngle, tailMotorAccelerationDelay_angle, tailMotorDecelerationDelay_angle, tailMotorPitchZeroAngle);

        int16_t angleDiff = servoSetpointAngle - servoAngle;
        if (ABS(angleDiff) > maxPhaseShift)
        {
            angleDiff = (int32_t)maxPhaseShift * angleDiff / ABS(angleDiff);
        }

        const int16_t futureServoAngle = constrain(servoAngle + angleDiff, TRI_TAIL_SERVO_ANGLE_MID - tailServoMaxAngle, TRI_TAIL_SERVO_ANGLE_MID + tailServoMaxAngle);
        uint16_t throttleMotorOutput = rcCommand[THROTTLE] - getCurrentMinthrottle();
        correction = (throttleMotorOutput * getPitchCorrectionAtTailAngle(DEGREES_TO_RADIANS(futureServoAngle / 10.0f))) - throttleMotorOutput;
    }

    return correction;
}

static uint16_t getServoValueAtAngle(servoParam_t *servoConf, uint16_t angle)
{
    int16_t servoMid = servoConf->middle;
    uint16_t servoValue;

    if (angle < TRI_TAIL_SERVO_ANGLE_MID)
    {
        int16_t servoMin = servoConf->min;
        servoValue = (int32_t)(angle - tailServoMaxAngle) * (servoMid - servoMin) / (TRI_TAIL_SERVO_ANGLE_MID - tailServoMaxAngle) + servoMin;
    }
    else if (angle > TRI_TAIL_SERVO_ANGLE_MID)
    {
        servoValue = (int32_t)(angle - TRI_TAIL_SERVO_ANGLE_MID) * (servoConf->max - servoMid) / tailServoMaxAngle + servoMid;
    }
    else
    {
        servoValue = servoMid;
    }

    return servoValue;
}

static float getPitchCorrectionAtTailAngle(float angle)
{
    return 1 / (sin_approx(angle) - cos_approx(angle) / tailServoThrustFactor);
}

static uint16_t getAngleFromYawCurveAtForce(int16_t force)
{
    if (force < yawForceCurve[0]) // No force that low
    {
        return TRI_TAIL_SERVO_ANGLE_MID - TRI_TAIL_SERVO_MAX_ANGLE;
    }
    else if (!(force < yawForceCurve[TRI_YAW_FORCE_CURVE_SIZE - 1])) // No force that high
    {
        return TRI_TAIL_SERVO_ANGLE_MID + TRI_TAIL_SERVO_MAX_ANGLE;
    }
    // Binary search: yawForceCurve[lower] <= force, yawForceCurve[higher] > force
    int32_t lower = 0, higher = TRI_YAW_FORCE_CURVE_SIZE - 1;
    while (higher > lower + 1)
    {
        const int32_t mid = (lower + higher) / 2;
        if (yawForceCurve[mid] > force)
        {
            higher = mid;
        }
        else
        {
            lower = mid;
        }
    }
    // Interpolating
    return TRI_TAIL_SERVO_ANGLE_MID - TRI_TAIL_SERVO_MAX_ANGLE + lower * 10 + (int32_t)(force - yawForceCurve[lower]) * 10 / (yawForceCurve[higher] - yawForceCurve[lower]);
}

static uint16_t getServoAngle(servoParam_t *servoConf, uint16_t servoValue)
{
    const int16_t midValue = servoConf->middle;
    const int16_t endValue = servoValue < midValue ? servoConf->min : servoConf->max;
    const int16_t endAngle = servoValue < midValue ? TRI_TAIL_SERVO_ANGLE_MID - tailServoMaxAngle : TRI_TAIL_SERVO_ANGLE_MID + tailServoMaxAngle;
    const int16_t servoAngle = (int32_t)(endAngle - TRI_TAIL_SERVO_ANGLE_MID) * (servoValue - midValue) / (endValue - midValue) + TRI_TAIL_SERVO_ANGLE_MID;
    return servoAngle;
}

static uint16_t getPitchCorrectionMaxPhaseShift(int16_t servoAngle,
        int16_t servoSetpointAngle,
        int16_t motorAccelerationDelayAngle,
        int16_t motorDecelerationDelayAngle,
        int16_t motorDirectionChangeAngle)
{
    uint16_t maxPhaseShift;

    if (((servoAngle > servoSetpointAngle) && (servoAngle >= (motorDirectionChangeAngle + motorAccelerationDelayAngle))) ||
        ((servoAngle < servoSetpointAngle) && (servoAngle <= (motorDirectionChangeAngle - motorAccelerationDelayAngle))))
    {
        // Motor is braking
        maxPhaseShift = ABS(servoAngle - motorDirectionChangeAngle) >= motorDecelerationDelayAngle ?
                motorDecelerationDelayAngle:
                ABS(servoAngle - motorDirectionChangeAngle);
    }
    else
    {
        // Motor is accelerating
        maxPhaseShift = motorAccelerationDelayAngle;
    }

    return maxPhaseShift;
}

static void virtualServoStep(float dT, servoParam_t *servoConf, uint16_t servoValue)
{
    const float angleSetPoint = getServoAngle(servoConf, servoValue) / 10.0f;
    const float dA = dT * tailServoSpeed; // Max change of an angle since last check
    if ( ABS(virtualServoAngle - angleSetPoint) < dA )
    {
        // At set-point after this moment
        virtualServoAngle = angleSetPoint;
    }
    else if (virtualServoAngle < angleSetPoint)
    {
        virtualServoAngle += dA;
    }
    else // virtualServoAngle > angleSetPoint
    {
        virtualServoAngle -= dA;
    }
}

static void triServoCalibrationStep()
{
    int16_t gyroSum;

    switch(servoCalib.state)
    {
    case IDLE:
        if (IS_RC_MODE_ACTIVE(BOXTAILTUNE))
        {
            ENABLE_FLIGHT_MODE(TAILTUNE_MODE);
            servoCalib.state = INIT;
            servoCalib.firstTime = true;
            servoCalib.baseTime = 0;
            servoCalib.counterLimit = 5;
            servoCalib.gyroPeakSum = 0;
        }
        break;
    case INIT:
        servoCalib.active = 1;
        servoCalib.numOfMeasurements = 0;
        servoCalib.counter = 0;
        servoCalib.servoPosition = DEFAULT_SERVO_MIDDLE;
        servoCalib.maxGyroSumIdle = 0;
        servoCalib.sumMeasurements = 0.0f;
        servoCalib.lowestMeasurement = 10000.0f;
        servoCalib.highestMeasurement = 0.0f;
        servoCalib.timestamp_us = micros();
        servoCalib.state = LISTEN_SILENCE;
        break;
    case LISTEN_SILENCE:
        if (IsDelayElapsed(servoCalib.timestamp_us, 500000))
        {
            if (!IsDelayElapsed(servoCalib.timestamp_us, 2000000))
            {
                servoCalib.maxGyroSumIdle = MAX(servoCalib.maxGyroSumIdle, getGyroSum());
            }
            else
            {
                if (servoCalib.maxGyroSumIdle > 15)
                {
                    // It was too loud, try again
                    servoCalib.maxGyroSumIdle = 0;
                    servoCalib.timestamp_us = micros();
                }
                else
                {
                    servoCalib.timestamp_us = micros();
                    servoCalib.state = SERVO_TO_START;
                    servoCalib.servoPosition = gpTailServoConf->max;
                    servoCalib.maxGyroSumIdle += 6;
                }
            }
        }
        break;
    case SERVO_TO_START:
        if (IsDelayElapsed(servoCalib.timestamp_us, 2000000))
        {
            servoCalib.timestamp_us = micros();
            servoCalib.state = DETECT_START;
            servoCalib.servoPosition = gpTailServoConf->min;
        }
        break;
    case DETECT_START:
        gyroSum = getGyroSum();
        if (servoCalib.firstTime)
        {
            if (gyroSum > servoCalib.maxGyroSumIdle)
            {
                servoCalib.timestamp_us = micros();
                servoCalib.state = DETECT_END;
            }
        }
        else
        {
            if (IsDelayElapsed(servoCalib.timestamp_us, servoCalib.baseTime))
            {
                if (gyroSum > servoCalib.gyroPeakSum)
                {
                    servoCalib.state = DETECT_END;
                }
            }
        }
        break;
    case DETECT_END:
        gyroSum = getGyroSum();

        if (servoCalib.firstTime)
        {
            servoCalib.gyroPeakSum = MAX(servoCalib.gyroPeakSum, gyroSum);
        }
        if (gyroSum <= servoCalib.maxGyroSumIdle)
        {
            servoCalib.counter++;
            if (servoCalib.counter > servoCalib.counterLimit)
            {
                if (servoCalib.firstTime)
                {
                    servoCalib.gyroPeakSum = (uint16_t)(servoCalib.gyroPeakSum * 0.5f);
                    servoCalib.baseTime = (uint32_t)((micros() - servoCalib.timestamp_us) * 0.5f);
                    servoCalib.firstTime = false;
                    servoCalib.counterLimit = 0;
                    servoCalib.state = INIT;
                }
                else
                {
                    // done
                    float time = (micros() - servoCalib.timestamp_us) / 1000000.0f;
                    float speed = (2.0f * tailServoMaxAngle / 10.0f) / time;
                    servoCalib.sumMeasurements += (uint16_t)speed;

                    servoCalib.lowestMeasurement = MIN(speed, servoCalib.lowestMeasurement);
                    servoCalib.highestMeasurement = MAX(speed, servoCalib.highestMeasurement);
                    servoCalib.numOfMeasurements++;

                    if (servoCalib.numOfMeasurements >= SERVO_CALIB_NUM_OF_MEAS)
                    {
                        servoCalib.state = CHECK_RESULTS;
                    }
                    else
                    {
                        servoCalib.timestamp_us = micros();
                        servoCalib.state = SERVO_TO_START;
                        servoCalib.servoPosition = gpTailServoConf->max;
                    }
                }
            }
        }
        else
        {
            if (servoCalib.counter > 0)
            {
                servoCalib.counter--;
            }
        }
        break;
    case CHECK_RESULTS:
        if ((servoCalib.highestMeasurement - servoCalib.lowestMeasurement) > 30.0f)
        {
            servoCalib.state = INIT;
        }
        else
        {
            gpMixerConfig->tri_tail_servo_speed = servoCalib.sumMeasurements / servoCalib.numOfMeasurements;
            saveConfigAndNotify();
            DISABLE_FLIGHT_MODE(TAILTUNE_MODE);
            servoCalib.active = 0;
            servoCalib.state = DONE;
            servoCalib.servoPosition = DEFAULT_SERVO_MIDDLE;
        }
        break;
    case DONE:
        if (!IS_RC_MODE_ACTIVE(BOXTAILTUNE))
        {
            servoCalib.state = IDLE;
        }
        break;
    }
}

