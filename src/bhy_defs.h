/**
 * Copyright (C) 2019 Bosch Sensortec GmbH
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 *
 * Neither the name of the copyright holder nor the names of the
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER
 * OR CONTRIBUTORS BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
 * OR CONSEQUENTIAL DAMAGES(INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE
 *
 * The information provided is believed to be accurate and reliable.
 * The copyright holder assumes no responsibility
 * for the consequences of use
 * of such information nor for any infringement of patents or
 * other rights of third parties which may result from its use.
 * No license is granted by implication or otherwise under any patent or
 * patent rights of the copyright holder.
 *
 * @file    bhy_defs.h
 * @date    30.1.2019
 * @version 0.1.0
 *
 */

#ifndef BHY_DEFS_H_
#define BHY_DEFS_H_

#if ARDUINO >= 100
 #include "Arduino.h"
 #include "stdio.h"
#else
 #include "WProgram.h"
#endif

#include <Wire.h>

#include <stdlib.h>

/*************************** C types headers *****************************/
#ifdef __KERNEL__
#include <linux/types.h>
#include <linux/kernel.h>
#else
#include <stdint.h>
#include <stddef.h>
#endif

/*************************** Common macros   *****************************/

#if !defined(UINT8_C) && !defined(INT8_C)
#define INT8_C(x)   S8_C(x)
#define UINT8_C(x)  U8_C(x)
#endif

#if !defined(UINT16_C) && !defined(INT16_C)
#define INT16_C(x)  S16_C(x)
#define UINT16_C(x) U16_C(x)
#endif

#if !defined(INT32_C) && !defined(UINT32_C)
#define INT32_C(x)  S32_C(x)
#define UINT32_C(x) U32_C(x)
#endif

#if !defined(INT64_C) && !defined(UINT64_C)
#define INT64_C(x)  S64_C(x)
#define UINT64_C(x) U64_C(x)
#endif

#ifndef NULL
#ifdef __cplusplus
#define NULL 0
#else
#define NULL ((void *) 0)
#endif
#endif

/*************************** C++ guard macro *****************************/
#ifdef __cplusplus
extern "C" {
#endif

/*************************** Sensor macros   *****************************/
/* Test for an endian machine */
#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
#define LITTLE_ENDIAN 1
#elif __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
#define BIG_ENDIAN    1
#else
#error "Code does not support Endian format of the processor"
#endif

/** Error Code Definitions */
#define BHY_OK                                     INT8_C(0)
#define BHY_E_NULL_PTR                             INT8_C(-1)
#define BHY_E_COM_FAIL                             INT8_C(-2)
#define BHY_E_DEV_NOT_FOUND                        INT8_C(-3)
#define BHY_E_OUT_OF_RANGE                         INT8_C(-4)
#define BHY_E_INVALID_INPUT                        INT8_C(-5)
#define BHY_E_ACCEL_ODR_BW_INVALID                 INT8_C(-6)
#define BHY_E_GYRO_ODR_BW_INVALID                  INT8_C(-7)
#define BHY_E_LWP_PRE_FLTR_INT_INVALID             INT8_C(-8)
#define BHY_E_LWP_PRE_FLTR_INVALID                 INT8_C(-9)
#define BHY_E_AUX_NOT_FOUND                        INT8_C(-10)
#define BHY_FOC_FAILURE                            INT8_C(-11)

#define BHY_E_INVALID_FIRMWARE                     INT8_C(-21)
#define BHY_E_RAMPATCH_MISMATCH                    INT8_C(-22)
#define BHY_E_CRC_MISMATCH                         INT8_C(-23)

#define BHY_E_PRODUCT_ID_MISMATCH                  INT8_C(-91)
#define BHY_E_UNKNOWN_ERROR                        INT8_C(-99)

/** BHY I2C Address */
#define BHY_I2C_ADDR                               UINT8_C(0x28)
#define BHY_I2C_ADDR2                              UINT8_C(0x29)

/** BHY Product ID */
#define BHY_PRODUCT_ID                             UINT8_C(0x83)

/** BHY Constants */
#define BHY_DISABLE                                UINT8_C(0)
#define BHY_ENABLE                                 UINT8_C(1)

/** ROM version constants */
#define BHY_ROM_VER_ANY                            UINT8_C(0x00)
#define BHY_ROM_VER_DI01                           UINT8_C(0x01)
#define BHY_ROM_VER_DI03                           UINT8_C(0x03)
#define BHY_ROM_VERSION_DI01                       UINT16_C(0x2112)
#define BHY_ROM_VERSION_DI03                       UINT16_C(0x2DAD)

/** Sensor ID Constants */
#define BHY_MAX_SENSOR_ID                          UINT8_C(32)
#define BHY_SID_WAKEUP_OFFSET                      UINT8_C(32)

/** Virtual Sensor IDs */
// Non-Wakeup sensors
#define BHY_SID_PADDING                            UINT8_C(0)
#define BHY_SID_ACCELEROMETER                      UINT8_C(1)
#define BHY_SID_MAGNETOMETER                       UINT8_C(2)
#define BHY_SID_ORIENTATION                        UINT8_C(3)
#define BHY_SID_GYROSCOPE                          UINT8_C(4)
#define BHY_SID_LIGHT                              UINT8_C(5)
#define BHY_SID_BAROMETER                          UINT8_C(6)
#define BHY_SID_TEMPERATURE                        UINT8_C(7)
#define BHY_SID_PROXIMITY                          UINT8_C(8)
#define BHY_SID_GRAVITY                            UINT8_C(9)
#define BHY_SID_LINEAR_ACCELERATION                UINT8_C(10)
#define BHY_SID_ROTATION_VECTOR                    UINT8_C(11)
#define BHY_SID_HUMIDITY                           UINT8_C(12)
#define BHY_SID_AMBIENT_TEMPERATURE                UINT8_C(13)
#define BHY_SID_UNCALIBRATED_MAGNETOMETER          UINT8_C(14)
#define BHY_SID_GAME_ROTATION_VECTOR               UINT8_C(15)
#define BHY_SID_UNCALIBRATED_GYROSCOPE             UINT8_C(16)
#define BHY_SID_SIGNIFICANT_MOTION                 UINT8_C(17)
#define BHY_SID_STEP_DETECTOR                      UINT8_C(18)
#define BHY_SID_STEP_COUNTER                       UINT8_C(19)
#define BHY_SID_GEOMAGNETIC_ROTATION_VECTOR        UINT8_C(20)
#define BHY_SID_HEART_RATE                         UINT8_C(21)
#define BHY_SID_TILT_DETECTOR                      UINT8_C(22)
#define BHY_SID_WAKE_GESTURE                       UINT8_C(23)
#define BHY_SID_GLANCE_GESTURE                     UINT8_C(24)
#define BHY_SID_PICKUP_GESTURE                     UINT8_C(25)
#define BHY_SID_CUS1                               UINT8_C(26)
#define BHY_SID_CUS2                               UINT8_C(27)
#define BHY_SID_CUS3                               UINT8_C(28)
#define BHY_SID_CUS4                               UINT8_C(29)
#define BHY_SID_CUS5                               UINT8_C(30)
#define BHY_SID_ACTIVITY                           UINT8_C(31)
// Wakeup sensors
#define BHY_SID_ACCELEROMETER_WAKEUP               UINT8_C(BHY_SID_ACCELEROMETER + BHY_SID_WAKEUP_OFFSET)
#define BHY_SID_MAGNETOMETER_WAKEUP                UINT8_C(BHY_SID_MAGNETOMETER + BHY_SID_WAKEUP_OFFSET)
#define BHY_SID_ORIENTATION_WAKEUP                 UINT8_C(BHY_SID_ORIENTATION + BHY_SID_WAKEUP_OFFSET)
#define BHY_SID_GYROSCOPE_WAKEUP                   UINT8_C(BHY_SID_GYROSCOPE + BHY_SID_WAKEUP_OFFSET)
#define BHY_SID_LIGHT_WAKEUP                       UINT8_C(BHY_SID_LIGHT + BHY_SID_WAKEUP_OFFSET)
#define BHY_SID_BAROMETER_WAKEUP                   UINT8_C(BHY_SID_BAROMETER + BHY_SID_WAKEUP_OFFSET)
#define BHY_SID_TEMPERATURE_WAKEUP                 UINT8_C(BHY_SID_TEMPERATURE + BHY_SID_WAKEUP_OFFSET)
#define BHY_SID_PROXIMITY_WAKEUP                   UINT8_C(BHY_SID_PROXIMITY + BHY_SID_WAKEUP_OFFSET)
#define BHY_SID_GRAVITY_WAKEUP                     UINT8_C(BHY_SID_GRAVITY + BHY_SID_WAKEUP_OFFSET)
#define BHY_SID_LINEAR_ACCELERATION_WAKEUP         UINT8_C(BHY_SID_LINEAR_ACCELERATION + BHY_SID_WAKEUP_OFFSET)
#define BHY_SID_ROTATION_VECTOR_WAKEUP             UINT8_C(BHY_SID_ROTATION_VECTOR + BHY_SID_WAKEUP_OFFSET)
#define BHY_SID_HUMIDITY_WAKEUP                    UINT8_C(BHY_SID_HUMIDITY + BHY_SID_WAKEUP_OFFSET)
#define BHY_SID_AMBIENT_TEMPERATURE_WAKEUP         UINT8_C(BHY_SID_AMBIENT_TEMPERATURE + BHY_SID_WAKEUP_OFFSET)
#define BHY_SID_UNCALIBRATED_MAGNETOMETER_WAKEUP   UINT8_C(BHY_SID_UNCALIBRATED_MAGNETOMETER + BHY_SID_WAKEUP_OFFSET)
#define BHY_SID_GAME_ROTATION_VECTOR_WAKEUP        UINT8_C(BHY_SID_GAME_ROTATION_VECTOR + BHY_SID_WAKEUP_OFFSET)
#define BHY_SID_UNCALIBRATED_GYROSCOPE_WAKEUP      UINT8_C(BHY_SID_UNCALIBRATED_GYROSCOPE + BHY_SID_WAKEUP_OFFSET)
#define BHY_SID_SIGNIFICANT_MOTION_WAKEUP          UINT8_C(BHY_SID_SIGNIFICANT_MOTION + BHY_SID_WAKEUP_OFFSET)
#define BHY_SID_STEP_DETECTOR_WAKEUP               UINT8_C(BHY_SID_STEP_DETECTOR + BHY_SID_WAKEUP_OFFSET)
#define BHY_SID_STEP_COUNTER_WAKEUP                UINT8_C(BHY_SID_STEP_COUNTER + BHY_SID_WAKEUP_OFFSET)
#define BHY_SID_GEOMAGNETIC_ROTATION_VECTOR_WAKEUP UINT8_C(BHY_SID_GEOMAGNETIC_ROTATION_VECTOR + BHY_SID_WAKEUP_OFFSET)
#define BHY_SID_HEART_RATE_WAKEUP                  UINT8_C(BHY_SID_HEART_RATE + BHY_SID_WAKEUP_OFFSET)
#define BHY_SID_TILT_DETECTOR_WAKEUP               UINT8_C(BHY_SID_TILT_DETECTOR + BHY_SID_WAKEUP_OFFSET)
#define BHY_SID_WAKE_GESTURE_WAKEUP                UINT8_C(BHY_SID_WAKE_GESTURE + BHY_SID_WAKEUP_OFFSET)
#define BHY_SID_GLANCE_GESTURE_WAKEUP              UINT8_C(BHY_SID_GLANCE_GESTURE + BHY_SID_WAKEUP_OFFSET)
#define BHY_SID_PICKUP_GESTURE_WAKEUP              UINT8_C(BHY_SID_PICKUP_GESTURE + BHY_SID_WAKEUP_OFFSET)
#define BHY_SID_CUS1_WAKEUP                        UINT8_C(BHY_SID_CUS1 + BHY_SID_WAKEUP_OFFSET)
#define BHY_SID_CUS2_WAKEUP                        UINT8_C(BHY_SID_CUS2 + BHY_SID_WAKEUP_OFFSET)
#define BHY_SID_CUS3_WAKEUP                        UINT8_C(BHY_SID_CUS3 + BHY_SID_WAKEUP_OFFSET)
#define BHY_SID_CUS4_WAKEUP                        UINT8_C(BHY_SID_CUS4 + BHY_SID_WAKEUP_OFFSET)
#define BHY_SID_CUS5_WAKEUP                        UINT8_C(BHY_SID_CUS5 + BHY_SID_WAKEUP_OFFSET)
#define BHY_SID_ACTIVITY_WAKEUP                    UINT8_C(BHY_SID_ACTIVITY + BHY_SID_WAKEUP_OFFSET)
// Other IDs
#define BHY_SID_DEBUG                              UINT8_C(245)
#define BHY_SID_TIMESTAMP_LSW_WAKEUP               UINT8_C(246)
#define BHY_SID_TIMESTAMP_MSW_WAKEUP               UINT8_C(247)
#define BHY_SID_META_EVENT_WAKEUP                  UINT8_C(248)
#define BHY_SID_BSX_C                              UINT8_C(249)
#define BHY_SID_BSX_B                              UINT8_C(250)
#define BHY_SID_BSX_A                              UINT8_C(251)
#define BHY_SID_TIMESTAMP_LSW                      UINT8_C(252)
#define BHY_SID_TIMESTAMP_MSW                      UINT8_C(253)
#define BHY_SID_META_EVENT                         UINT8_C(254)
// Internal Use
#define BHY_SID_NONE                               UINT8_C(255)

/** Enumerated Virtual Sensor IDs */
typedef enum {
    BHY_VS_NOT_USED                    = BHY_SID_PADDING,
    BHY_VS_ACCELEROMETER               = BHY_SID_ACCELEROMETER,
    BHY_VS_GEOMAGNETIC_FIELD           = BHY_SID_MAGNETOMETER,
    BHY_VS_ORIENTATION                 = BHY_SID_ORIENTATION,
    BHY_VS_GYROSCOPE                   = BHY_SID_GYROSCOPE,
    BHY_VS_LIGHT                       = BHY_SID_LIGHT,
    BHY_VS_PRESSURE                    = BHY_SID_BAROMETER,
    BHY_VS_TEMPERATURE                 = BHY_SID_TEMPERATURE,
    BHY_VS_PROXIMITY                   = BHY_SID_PROXIMITY,
    BHY_VS_GRAVITY                     = BHY_SID_GRAVITY,
    BHY_VS_LINEAR_ACCELERATION         = BHY_SID_LINEAR_ACCELERATION,
    BHY_VS_ROTATION_VECTOR             = BHY_SID_ROTATION_VECTOR,
    BHY_VS_RELATIVE_HUMIDITY           = BHY_SID_HUMIDITY,
    BHY_VS_AMBIENT_TEMPERATURE         = BHY_SID_AMBIENT_TEMPERATURE,
    BHY_VS_MAGNETIC_FIELD_UNCALIBRATED = BHY_SID_UNCALIBRATED_MAGNETOMETER,
    BHY_VS_GAME_ROTATION_VECTOR        = BHY_SID_GAME_ROTATION_VECTOR,
    BHY_VS_GYROSCOPE_UNCALIBRATED      = BHY_SID_UNCALIBRATED_GYROSCOPE,
    BHY_VS_SIGNIFICANT_MOTION          = BHY_SID_SIGNIFICANT_MOTION,
    BHY_VS_STEP_DETECTOR               = BHY_SID_STEP_DETECTOR,
    BHY_VS_STEP_COUNTER                = BHY_SID_STEP_COUNTER,
    BHY_VS_GEOMAGNETIC_ROTATION_VECTOR = BHY_SID_GEOMAGNETIC_ROTATION_VECTOR,
    BHY_VS_HEART_RATE                  = BHY_SID_HEART_RATE,
    BHY_VS_TILT                        = BHY_SID_TILT_DETECTOR,
    BHY_VS_WAKE_GESTURE                = BHY_SID_WAKE_GESTURE,
    BHY_VS_GLANCE_GESTURE              = BHY_SID_GLANCE_GESTURE,
    BHY_VS_PICKUP_GESTURE              = BHY_SID_PICKUP_GESTURE,
    BHY_VS_CUS1                        = BHY_SID_CUS1,
    BHY_VS_CUS2                        = BHY_SID_CUS2,
    BHY_VS_CUS3                        = BHY_SID_CUS3,
    BHY_VS_CUS4                        = BHY_SID_CUS4,
    BHY_VS_CUS5                        = BHY_SID_CUS5,
    BHY_VS_ACTIVITY_RECOGNITION        = BHY_SID_ACTIVITY,
    BHY_VS_INVALID
} bhyVirtualSensor;

/** Enumerated Page/Parameter Values */
typedef enum {
    BHY_ALGORITHM_FINISH            = 0,
    // Must be sent when finished accessing algorithm page
    BHY_SYSTEM_PAGE                 = 1,
    BHY_ALGORITHM_PAGE              = 2,
    BHY_SENSORS_PAGE                = 3,
    BHY_CUSTOM_12                   = 12,
    BHY_CUSTOM_13                   = 13,
    BHY_CUSTOM_14                   = 14,
    BHY_SOFT_PASS_THROUGH_PAGE      = 15
} bhyPage;

typedef enum {
    BHY_META_EVENT_CONTROL          = 1,
    BHY_FIFO_CONTROL                = 2,
    BHY_SENSOR_STATUS_BANK_0        = 3,
    BHY_SENSOR_STATUS_BANK_1        = 4,
    BHY_SENSOR_STATUS_BANK_2        = 5,
    BHY_SENSOR_STATUS_BANK_3        = 6,
    // IDs 7-28 are reserved
    BHY_META_EVENT_CONTROL_WAKEUP   = 29,
    BHY_HOST_IRQ_TIMESTAMP          = 30,
    BHY_PHYSICAL_SENSOR_STATUS      = 31,
    BHY_PHYSICAL_SENSOR_DETAILS_0   = 32,
    BHY_PHYSICAL_SENSOR_DETAILS_1   = 33,
} bhySystemPageParameter;

/** Enumerated Flush Values */
#define VS_FLUSH_ALL 0xFF

typedef enum {
    BHY_FLUSH_NONE      = 0x00,
    BHY_FLUSH_ONE       = 0x01,
    BHY_FLUSH_ALL       = VS_FLUSH_ALL
} bhyFlush;

/** The data types used.
    This follows section 15 of the BHI160 datasheet the order of this enumeration.
    This is important, do not change it. */
typedef enum {
    BHY_PADDING_TYPE                = 0,
    BHY_QUATERNION_TYPE             = 1,
    BHY_VECTOR_TYPE                 = 2,
    BHY_SCALAR_TYPE_U8              = 3,
    BHY_SCALAR_TYPE_U16             = 4,
    BHY_SCALAR_TYPE_S16             = 5,
    BHY_SCALAR_TYPE_U24             = 6,
    BHY_SENSOR_EVENT_TYPE           = 7,
    BHY_VECTOR_TYPE_UNCALIBRATED    = 8,
    BHY_META_TYPE                   = 9,
    BHY_BSX_TYPE                    = 10,
    BHY_DEBUG_TYPE                  = 11,
    BHY_CUSTOM_TYPE_1               = 12,
    BHY_CUSTOM_TYPE_2               = 13,
    BHY_CUSTOM_TYPE_3               = 14,
    BHY_CUSTOM_TYPE_4               = 15,
    BHY_CUSTOM_TYPE_5               = 16,
    BHY_ACTIVITY_TYPE               = 17,
    BHY_NO_DATA_TYPE,
    BHY_INVALID_DATA_TYPE
} bhyDataType;

/** Enumerated Meta Event Types */
typedef enum {
    BHY_META_TYPE_NOT_USED                = 0,
    BHY_META_TYPE_FLUSH_COMPLETE          = 1,
    BHY_META_TYPE_SAMPLE_RATE_CHANGED     = 2,
    BHY_META_TYPE_POWER_MODE_CHANGED      = 3,
    BHY_META_TYPE_ERROR                   = 4,
    BHY_META_TYPE_ALGORITHM               = 5,
    // IDs 6-10 are reserved
    BHY_META_TYPE_SENSOR_ERROR            = 11,
    BHY_META_TYPE_FIFO_OVERFLOW           = 12,
    BHY_META_TYPE_DYNAMIC_RANGE_CHANGED   = 13,
    BHY_META_TYPE_FIFO_WATERMARK          = 14,
    BHY_META_TYPE_SELF_TEST_RESULTS       = 15,
    BHY_META_TYPE_INITIALIZED             = 16,
    BHY_META_TYPE_INVALID
} bhyMetaEventType;

/** Enumerated Host Interface Control Bit Values */
typedef enum {
    BHY_ALGORITHM_STANDBY_REQUEST_BIT       = 0,
    BHY_ABORT_TRANSFER_BIT                  = 1,
    BHY_UPDATE_TRANSFER_COUNT_BIT           = 2,
    BHY_WAKEUP_INTERRUPT_DISABLE_BIT        = 3,
    BHY_NED_COORDINATES_BIT                 = 4,
    BHY_AP_SUSPENDED_BIT                    = 5,
    BHY_REQUEST_SENSOR_SELF_TEST_BIT        = 6,
    BHY_NON_WAKEUP_INTERRUPT_DISABLE_BIT    = 7
} bhyHICBit;

/** Enumerated Debug Levels */
typedef enum
{
    BHY_NONE = 0,
    BHY_ERROR,
    BHY_DEBUG,
    BHY_INFORMATIVE,
    BHY_ALL,
    BHY_INTERNAL
} bhyDebugLevel;

/** For compatibility with past drivers */
typedef int8_t (*bhyComFptr)(TwoWire* wire, uint8_t devId, uint8_t regAddr, uint8_t *data, uint16_t length);
typedef void (*bhyDelayFptr)(uint32_t period);

typedef enum
{
    BHY_PHYSICAL_SENSOR_INDEX_ACC = 0,
    BHY_PHYSICAL_SENSOR_INDEX_MAG,
    BHY_PHYSICAL_SENSOR_INDEX_GYRO,
    BHY_PHYSICAL_SENSOR_COUNT
} bhyPhysicalSensorIndexType;

/** Definition of data type structures */
typedef struct
{
    float x;
    float y;
    float z;
    float w;
    float accuracyRadians;
} bhyQuaternion;

typedef struct
{
    float x;
    float y;
    float z;
    uint8_t status;
} bhyVector;

typedef struct
{
    int16_t x;
    int16_t y;
    int16_t z;
    int16_t x_bias;
    int16_t y_bias;
    int16_t z_bias;
    uint8_t status;
} bhyVectorUncalib;

typedef union
{
    uint16_t value;
    struct
    {
        #if LITTLE_ENDIAN == 1
        // still activity ended
        uint8_t still_ended : 1;
        // walk activity ended
        uint8_t walking_ended : 1;
        // running activity ended
        uint8_t running_ended : 1;
        // on bicycle activity ended
        uint8_t bicycle_ended : 1;
        // in vehicle activity ended
        uint8_t vehicle_ended : 1;
        // tilting activity ended
        uint8_t tilting_ended : 1;
        // reserved
        uint8_t  : 2;
        uint8_t  : 0; // ensure the start of a new byte
        // still activity ended
        uint8_t still_started : 1;
        // walk activity ended
        uint8_t walking_started : 1;
        // running activity ended
        uint8_t running_started : 1;
        // on bicycle activity ended
        uint8_t bicycle_started : 1;
        // in vehicle activity ended
        uint8_t vehicle_started : 1;
        // tilting activity ended
        uint8_t tilting_started : 1;
        // reserved
        uint8_t  : 2;
        #elif BIG_ENDIAN == 1
        // still activity ended
        uint8_t still_started : 1;
        // walk activity ended
        uint8_t walking_started : 1;
        // running activity ended
        uint8_t running_started : 1;
        // on bicycle activity ended
        uint8_t bicycle_started : 1;
        // in vehicle activity ended
        uint8_t vehicle_started : 1;
        // tilting activity ended
        uint8_t tilting_started : 1;
        // reserved
        uint8_t  : 2;
        uint8_t  : 0; // ensure the start of a new byte
        // still activity ended
        uint8_t still_ended : 1;
        // walk activity ended
        uint8_t walking_ended : 1;
        // running activity ended
        uint8_t running_ended : 1;
        // on bicycle activity ended
        uint8_t bicycle_ended : 1;
        // in vehicle activity ended
        uint8_t vehicle_ended : 1;
        // tilting activity ended
        uint8_t tilting_ended : 1;
        // reserved
        uint8_t  : 2;
        #endif
    };
} bhyActivityEvent;

typedef struct
{
    uint8_t id;
    bhyMetaEventType type;
    union
    {
        struct
        {
        #if LITTLE_ENDIAN == 1
            uint8_t info;
            uint8_t data;
        #elif BIG_ENDIAN == 1
            uint8_t data;
            uint8_t info;
        #endif
        };
        uint16_t uint16;
        int16_t int16;
    };
} bhyMetaEvent;

typedef struct
{
    int32_t x;
    int32_t y;
    int32_t z;
    uint32_t timestamp;
} bhyBSX;

typedef struct
{
    int16_t deltaX;
    int16_t deltaY;
    int16_t deltaZ;
    int16_t confidencelevel;
    uint16_t direction;
    uint16_t stepCount;
} bhyPDR;

/*!
 *   @brief Sensor configuration
 *   This structure holds sensor configuration
 */
typedef struct
{
    uint16_t sampleRate;
    uint16_t maxReportLatency;
    uint16_t changeSensitivity;
    uint16_t dynamicRange;
} bhySensorConfiguration;

typedef struct
{
    // the sampling rate
    uint16_t sampleRate;
    // the dynamic range
    uint16_t dynamicRange;
    // the flag
    uint8_t flag;
} bhyPhysicalStatus;

/*!
 *   @brief Sensor information
 *   This structure holds sensor information
 */
typedef struct
{
    // sensor type
    uint8_t type;
    // driver id
    uint8_t driver_id;
    // driver version
    uint8_t driver_version;
    // the power example: 0.1mA
    uint8_t power;
    // the maxim range of sensor data in SI units
    uint16_t max_range;
    // the no of bit resolution of underlying sensor
    uint16_t resolution;
    // the maximum rate in Hz
    uint16_t max_rate;
    // the fifo size
    uint16_t fifo_reserved;
    // the entire fifo size
    uint16_t fifo_max;
    // the no of bytes sensor data packet
    uint8_t event_size;
    // the minimum rate in Hz
    uint8_t min_rate;
} bhySensorInformation;

typedef union
{
    uint8_t value;
    struct
    {
        // if data is available
        uint8_t data_available : 1;
        // if sensor did not acknowledge transfer
        uint8_t nack : 1;
        // if there is a device ID mismatch
        uint8_t id_error : 1;
        // if there is a transient error
        uint8_t transient_error : 1;
        // if there was a FIFO overflow
        uint8_t data_lost : 1;
        // the power mode
        uint8_t power_mode : 3;
        // 0: Not Present
        // 1: Power Down
        // 2: Suspend
        // 3: Self-Test
        // 4: Interrupt Motion
        // 5: One Shot
        // 6: Low Power Active
        // 7: Active
    };
} bhySensorStatus;

typedef union
{
    uint8_t value;
    struct
    {
        // reset (ignored)
        uint8_t reset : 1;
        // host algo status
        uint8_t algorithm_status : 1;
        // host interface id
        uint8_t host_interface_id : 3;
        // host algo id (0 for BSX Fusion Library)
        uint8_t algorithm_id : 3;
    };
} bhyHostStatus;

typedef union
{
    uint8_t value;
    struct
    {
        // host interrupt status, the state of the host interrupt GPIO pin
        uint8_t interrupt_status : 1;
        // wakeup watermark interrupt status
        uint8_t wakeup_water_mark : 1;
        // wakeup latency interrupt status
        uint8_t wakeup_latency : 1;
        // wakeup immediate interrupt status
        uint8_t wakeup_immediate : 1;
        // non wakeup watermark interrupt status
        uint8_t non_wakeup_water_mark : 1;
        // non wakeup latency interrupt status
        uint8_t non_wakeup_latency : 1;
        // non wakeup immediate status
        uint8_t non_wakeup_immediate : 1;
    };
} bhyInterruptStatus;

typedef union
{
    uint8_t value;
    struct
    {
        // eeprom detected status
        uint8_t eeprom_detected : 1;
        // eeprom upload done status
        uint8_t ee_upload_done : 1;
        // eeprom upload error status
        uint8_t ee_upload_error : 1;
        // firmware idle status
        uint8_t firmware_idle : 1;
        // no eeprom detected status
        uint8_t no_eeprom : 1;
    };
} bhyChipStatus;

typedef union
{
    uint8_t value;
    struct
    {
        // algorithm standby request status
        uint8_t alg_standby_request : 1;
        // host interrupt control status
        uint8_t abort_transfer : 1;
        // update transfer control status
        uint8_t update_transfer_cnt : 1;
        // wakeup fifo host interrupt disable status
        uint8_t wakeup_fifo_intr_disable : 1;
        // ned coordinates status
        uint8_t ned_coordinates : 1;
        // AP suspended status
        uint8_t ap_suspend : 1;
        // self test status
        uint8_t sensor_selftest : 1;
        // non wakeup fifo host interrupt disable status
        uint8_t non_wakeup_fifo_intr_disable : 1;
    };
} bhyHostInterfaceControl;

typedef struct
{
    // the value of I2C slave address
    uint8_t address;
    // the value of start register
    uint8_t start;
    // the value of read length
    uint8_t length;
    // the value of completion status
    // 0 = transfer in progress or not issued yet
    // 1 = transfer successful
    // 2 = I2C Nack or error
    uint8_t status;
    // the value of returned register values
    uint8_t data[4];
} bhySoftPassThrough;

/*!
 *   @brief sensor status bank structure
 *   This structure holds sensor status bank information
 */
typedef union
{
    uint8_t value;
    struct
    {
        // the status bank data available information
        uint8_t data_available : 1;
        // the status bank i2c NACK information
        uint8_t i2c_nack : 1;
        // the status bank device id error information
        uint8_t device_id_error : 1;
        // the status bank transient error information
        uint8_t transient_error : 1;
        // the status bank data loss information
        uint8_t data_lost : 1;
        // the status bank sensor power mode information
        uint8_t sensor_power_mode : 3;
    };
} bhySensorStatusBank;

/** Definition of callback functions */
typedef void (*bhyCallbackFptrTypeOnly)(bhyVirtualSensor);
typedef void (*bhyCallbackFptrU8)(uint8_t, bhyVirtualSensor);
typedef void (*bhyCallbackFptrU16)(uint16_t, bhyVirtualSensor);
typedef void (*bhyCallbackFptrS16)(int16_t, bhyVirtualSensor);
typedef void (*bhyCallbackFptrBytes)(uint8_t *, uint8_t length, bhyVirtualSensor);
typedef void (*bhyCallbackFptrQuaternion)(bhyQuaternion, bhyVirtualSensor);
typedef void (*bhyCallbackFptrVector)(bhyVector, bhyVirtualSensor);
typedef void (*bhyCallbackFptrVectorUncalib)(bhyVectorUncalib*, bhyVirtualSensor);
typedef void (*bhyCallbackFptrFloat)(float, bhyVirtualSensor);
typedef void (*bhyCallbackFptrActivity)(bhyActivityEvent, bhyVirtualSensor);

typedef void (*bhyCallbackFptrBSX)(bhyBSX *, bhyVirtualSensor);
typedef void (*bhyCallbackFptrPDR)(bhyPDR *, bhyVirtualSensor);

typedef void (*bhyCallbackFptrTimestamp)(uint16_t);

typedef void (*bhyCallbackFptrMetaEvent)(bhyMetaEvent *, bhyMetaEventType);

/** Union used internally */
union bhyCallbackFptr
{
    void *invalid;
    bhyCallbackFptrTypeOnly typeOnly;
    bhyCallbackFptrU8 uint8;
    bhyCallbackFptrU16 uint16;
    bhyCallbackFptrS16 int16;
    bhyCallbackFptrBytes bytes;
    bhyCallbackFptrQuaternion quaternion;
    bhyCallbackFptrVector vector;
    bhyCallbackFptrVectorUncalib vectorUncalib;
    bhyCallbackFptrActivity activity;
    bhyCallbackFptrFloat scaledFloat;
};

typedef struct
{
    uint32_t seconds;
    uint32_t nanoseconds;
} bhyTimestamp;

/* these FIFO sizes are dependent on the type enumeration  */
#define BHY_FRAME_SIZE_PADDING        1
#define BHY_FRAME_SIZE_QUATERNION     11
#define BHY_FRAME_SIZE_VECTOR         8
#define BHY_FRAME_SIZE_SCALAR_U8      2
#define BHY_FRAME_SIZE_SCALAR_U16     3
#define BHY_FRAME_SIZE_SCALAR_S16     3
#define BHY_FRAME_SIZE_SCALAR_U24     4
#define BHY_FRAME_SIZE_SENSOR_EVENT   1
#define BHY_FRAME_SIZE_UNCALIB_VECTOR 14
#define BHY_FRAME_SIZE_META_EVENT     4
#define BHY_FRAME_SIZE_BSX            17
#define BHY_FRAME_SIZE_DEBUG          14

/* set default custom sensor packet size to 1, same as padding */
#define BHY_FRAME_SIZE_CUS1           1
#define BHY_FRAME_SIZE_CUS2           1
#define BHY_FRAME_SIZE_CUS3           1
#define BHY_FRAME_SIZE_CUS4           1
#define BHY_FRAME_SIZE_CUS5           1

/*************************** C++ guard macro *****************************/
#ifdef __cplusplus
}
#endif

#endif // BHY_DEFS_H_
