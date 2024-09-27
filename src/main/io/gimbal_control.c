/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>
#include <math.h>
#include <ctype.h>
#include <string.h>

#include "platform.h"

#if defined(USE_GIMBAL)

#include "build/debug.h"

#include "common/crc.h"
#include "common/maths.h"

#include "drivers/time.h"

#include "io/serial.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"
#include "pg/gimbal.h"

#include "scheduler/scheduler.h"

/* The setting below accommodate both -ve and +ve setting so the gimbal can be mounted either  way up.
 *
 * |------------------------|-----|------------|----------------------------------------------------------------------------------------|
 * | Setting                | Def | Range      | Purpose                                                                                |
 * |------------------------|-----|------------|----------------------------------------------------------------------------------------|
 * | gimbal_roll_gain       | 40  | -100 - 100 | Adjusts amount of roll in %                                                            |
 * | gimbal_roll_offset     | 0   | -100 - 100 | Adjust roll position for neutral roll input                                            |
 * | gimbal_pitch_thr_gain  | 10  | -100 - 100 | Adjusts amount of pitch increase for throttle input in %                               |
 * | gimbal_pitch_low_gain  | 20  | -100 - 100 | Adjusts amount of pitch increase for low pitch input in %                              |
 * | gimbal_pitch_high_gain | -10 | -100 - 100 | Adjusts amount of pitch decrease for high pitch input in %                             |
 * | gimbal_pitch_offset    | 10  | -100 - 100 | Adjust pitch position for neutral pitch input                                          |
 * | gimbal_yaw_gain        | 20  | -100 - 100 | Adjusts amount of yaw in %                                                             |
 * | gimbal_yaw_offset      | 0   | -100 - 100 | Adjust yaw position for neutral yaw input                                              |
 * | gimbal_stabilisation   | 0   |    0 - 7   | 0 no stabilisation, 1 pitch stabilisation, 2 roll/pitch stabilisation, 3 - 7 reserved  |
 * | gimbal_sensitivity     | 15  |  -16 - 15  | With higher values camera more rigidly tracks quad motion                              |
 * |------------------------|-----|------------|----------------------------------------------------------------------------------------|
 *
 * To enable the gimbal control on a port set bit 18 thus:
 *
 *  serial <port> 262144 115200 57600 0 115200
 */

#define GIMBAL_CMD_NONE               0x00  //empty order
#define GIMBAL_CMD_ACCE_CALIB         0x01  //Acceleration Calibration
#define GIMBAL_CMD_GYRO_CALIB         0x02  //Gyro Calibration
#define GIMBAL_CMD_MAGN_CALIB         0x03  //Calibration of magnetometers
#define GIMBAL_CMD_AHRS_GZERO         0x04  //Zeroing the attitude angle
#define GIMBAL_CMD_QUIT_CALIB         0x05  //Exit current calibration

// Command Code Return Macro Definition
#define GIMBAL_STATUS_ERR             0x80

// This packet is sent to control the gimbal mode, sensivity and position
#define GIMBAL_CMD_S  0x5AA5
typedef struct {
    uint16_t opcode;
    unsigned mode:3;     // Mode [0~7] Only 0 1 2 modes are supported
    signed   sens:5;     // Sensitivity [-16~15]
    unsigned padding:4;
    signed   roll:12;   // Roll angle [-2048~2047] - ±180deg
    signed   pitch:12;  // Pich angle [-2048~2047] - ±180deg
    signed   yaw:12;    // Yaw angle [-2048~2047]  - ±180deg
    uint16_t crc;
}  __attribute__ ((__packed__)) gimbalCmd_t;

/* This packet is sent by the user to the head chase to realize the head
 * chase calibration and reset function
 */
#define GIMBAL_CMD_L  0x6EC5
typedef struct {
    uint16_t opcode;
    uint8_t  cmd;        // Command
    unsigned pwm:5;      // Pulse width data [0~31] => [0%~100%]
    unsigned iom:3;      // GPIO mode [0~7]
    unsigned mode:3;     // Mode [0~7] Only 0 1 2 modes are supported
    signed   sens:5;     // Sensitivity [-16~15]
    struct
    {
        unsigned chan:3;  // Channel [0~7] [CH56,CH57,CH58,CH67,CH68,CH78,CH78,CH78]
        unsigned revs:2;  // Reverse [0~3] [Normal, Horizontal Reverse, Vertical Reverse, All Reverse]
        unsigned rngx:2;  // Range [0~3] [90 degrees, 120 degrees, 180 degrees, 360 degrees]
        unsigned rngy:2;  // Range [0~3] [60 degrees, 90 degrees, 120 degrees, 180 degrees]
        signed   zerx:6;  // Zero [-32~31] [Resolution:5us]
        signed   zery:6;  // Zero [-32~31] [Resolution:5us]
        unsigned padding:11;
    } ppm;
    uint8_t padding2[5];
    uint16_t crc;
}  __attribute__ ((__packed__)) gimbalCal_t;

// Status reponse packet
#define GIMBAL_STAT  0x913A
typedef struct {
    uint16_t opcode;
    uint8_t cmd;        // Command response status
    uint8_t ctyp;       // Calibration type [0:Idle 1:Acceleration calibration 2:Gyroscope calibration 3:Magnetometer calibration]
    uint8_t cprg;       // Calibration progress [0%~100%]
    uint8_t cerr;       // Calibration error [0%~100%]
    uint8_t padding[8];
    uint16_t crc;
}  __attribute__ ((__packed__)) gimbalCalStatus_t;

#define GIMBAL_SET_MIN      -500
#define GIMBAL_SET_MAX      500
#define GIMBAL_ROLL_MIN     -500
#define GIMBAL_ROLL_MAX     500
#define GIMBAL_PITCH_MIN    -1150
#define GIMBAL_PITCH_MAX    1750
#define GIMBAL_YAW_MIN      -2047
#define GIMBAL_YAW_MAX      2047

static gimbalCmd_t gimbalCmd = {0};
static serialPort_t *gimbalSerialPort = NULL;

static uint16_t gimbalCrc(uint8_t *buf, uint32_t size)
{
    return __builtin_bswap16(crc16_ccitt_update(0x0000, buf, size));
}

// Set the gimbal position on each axis in a ±500 range
bool gimbalSet(int16_t roll, int16_t pitch, int16_t yaw)
{
    DEBUG_SET(DEBUG_GIMBAL, 0, roll);
    DEBUG_SET(DEBUG_GIMBAL, 1, pitch);
    DEBUG_SET(DEBUG_GIMBAL, 2, yaw);

    if (!gimbalSerialPort) {
        return false;
    }

    // Scale the incoming ±500 range to the max values accepted by the gimbal
    roll  = scaleRange(roll,  GIMBAL_SET_MIN, GIMBAL_SET_MAX, GIMBAL_ROLL_MIN,  GIMBAL_ROLL_MAX);
    pitch = scaleRange(pitch, GIMBAL_SET_MIN, GIMBAL_SET_MAX, GIMBAL_PITCH_MAX, GIMBAL_PITCH_MIN);
    yaw   = scaleRange(yaw,   GIMBAL_SET_MIN, GIMBAL_SET_MAX, GIMBAL_YAW_MIN,   GIMBAL_YAW_MAX);

    // Constrain to catch any incoming out of range values
    gimbalCmd.roll = constrain(roll, GIMBAL_ROLL_MIN, GIMBAL_ROLL_MAX);
    gimbalCmd.pitch = constrain(pitch, GIMBAL_PITCH_MIN, GIMBAL_PITCH_MAX);
    gimbalCmd.yaw = constrain(yaw, GIMBAL_YAW_MIN, GIMBAL_YAW_MAX);

    DEBUG_SET(DEBUG_GIMBAL, 3, gimbalCmd.roll);
    DEBUG_SET(DEBUG_GIMBAL, 4, gimbalCmd.pitch);
    DEBUG_SET(DEBUG_GIMBAL, 5, gimbalCmd.yaw);

    gimbalCmd.mode = gimbalTrackConfig()->gimbal_stabilisation;
    gimbalCmd.sens = gimbalTrackConfig()->gimbal_sensitivity;

    uint16_t crc = gimbalCrc((uint8_t *)&gimbalCmd, sizeof(gimbalCmd) - 2);

    gimbalCmd.crc = crc;

    return true;
}

// Set the gimbal position on each axis in a ±500 range with scale and offset applied
bool gimbalTrack(int16_t throttle, int16_t roll, int16_t pitch, int16_t yaw)
{
    int16_t gimbal_roll;
    int16_t gimbal_pitch;
    int16_t gimbal_yaw;

    // Apply scale an offset, each of which are expressed as a percentage
    gimbal_roll = (roll * gimbalTrackConfig()->gimbal_roll_gain / 100) + 5 * gimbalTrackConfig()->gimbal_roll_offset;
    gimbal_yaw = (yaw * gimbalTrackConfig()->gimbal_yaw_gain / 100) + 5 * gimbalTrackConfig()->gimbal_yaw_offset;

    if (pitch < 0) {
        gimbal_pitch = pitch * gimbalTrackConfig()->gimbal_pitch_low_gain / 100;
    } else {
        gimbal_pitch = pitch * gimbalTrackConfig()->gimbal_pitch_high_gain / 100;
    }

    gimbal_pitch += 5 * gimbalTrackConfig()->gimbal_pitch_offset - (throttle * gimbalTrackConfig()->gimbal_pitch_thr_gain / 100);

    return gimbalSet(gimbal_roll, gimbal_pitch, gimbal_yaw);
}

// Gimbal updates should be sent at 100Hz or the gimbal will self center after approx. 2 seconds
void gimbalUpdate(timeUs_t currentTimeUs)
{
    UNUSED(currentTimeUs);

    if (!gimbalSerialPort) {
        setTaskEnabled(TASK_GIMBAL, false);
        return;
    }
    serialWriteBuf(gimbalSerialPort, (uint8_t *)&gimbalCmd, sizeof(gimbalCmd));
}

bool gimbalInit(void)
{
    const serialPortConfig_t *portConfig = findSerialPortConfig(FUNCTION_GIMBAL);
    if (!portConfig) {
        return false;
    }

    // Serial communications is 115200 8N1
    gimbalSerialPort = openSerialPort(portConfig->identifier, FUNCTION_GIMBAL,
                                      NULL, NULL,
                                      115200, MODE_RXTX, SERIAL_STOPBITS_1 | SERIAL_PARITY_NO);

    gimbalCmd.opcode = GIMBAL_CMD_S;

    // Set gimbal initial position
    gimbalSet(0, 0, 0);

    return true;
}

#endif
