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

#ifndef SRC_MAIN_FLIGHT_MIXER_TRICOPTER_H_
#define SRC_MAIN_FLIGHT_MIXER_TRICOPTER_H_

#define TAIL_THRUST_FACTOR_MIN  (10)
#define TAIL_THRUST_FACTOR_MAX  (400)

#define TAIL_THRUST_FACTOR_MIN_FLOAT  (TAIL_THRUST_FACTOR_MIN / 10.0f)
#define TAIL_THRUST_FACTOR_MAX_FLOAT  (TAIL_THRUST_FACTOR_MAX / 10.0f)

/** @brief Servo feedback sources. */
typedef enum {
    TRI_SERVO_FB_VIRTUAL = 0,  ///< Virtual servo, no physical feedback signal from servo
    TRI_SERVO_FB_RSSI,         ///< Feedback signal from RSSI ADC
    TRI_SERVO_FB_CURRENT,      ///< Feedback signal from CURRENT ADC
    TRI_SERVO_FB_EXT1,         ///< Feedback signal from EXT1 ADC
} triServoFeedbackSource_e;

/** @brief Initialize tricopter specific mixer functionality.
 *
 *  @param pTailServoConfig Pointer to tail servo configuration
 *  when in tricopter mixer mode.
 *  @param pTailServo Pointer to tail servo output value.
 *  @param pMixerConfig Pointer to mixer configuration.
 *  @return Void.
 */
void triInitMixer(servoParam_t *pTailServoConfig,
        int16_t *pTailServo,
        mixerConfig_t *pMixerConfig);

/** @brief Get current tail servo angle.
 *
 *  @return Servo angle in decidegrees.
 */
uint16_t triGetCurrentServoAngle();

/** @brief Perform tricopter mixer actions.
 *
 *  @return Void.
 */
void triServoMixer();

/** @brief Get amount of motor correction that must be applied
 * for given motor.
 *
 * Needed correction amount is calculated based on current servo
 * position to maintain pitch axis attitude.
 *
 *  @return Amount of motor correction that must be added to
 *  motor output.
 */
int16_t triGetMotorCorrection(uint8_t motorIndex);

/** @brief Should tail servo be active when unarmed.
 *
 *  @return true is should, otherwise false.
 */
_Bool triEnableServoUnarmed(void);

#endif /* SRC_MAIN_FLIGHT_MIXER_TRICOPTER_H_ */
