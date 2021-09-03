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
 * @file    bhy.h
 * @date    30.1.2019
 * @version 0.1.0
 *
 */

#ifndef BHY_H_
#define BHY_H_

#include "bhy_defs.h"

#ifndef DEBUG_MODE
//#define DEBUG_MODE
#endif

/** BHY Size Definitions */
#define SENSOR_CALLBACK_LIST_NUM             UINT8_C(64) // 1-64 are standard types
#define METAEVENT_CALLBACK_LIST_NUM          UINT8_C(32) // 1-16 are standard types, 17-32 are for wake-up
#define TIMESTAMP_CALLBACK_LIST_NUM          UINT8_C(2) // 1 is LSW

#define BHY_FIFO_BUFFER_SIZE                 INT16_C(256) // Don't change this
#define BHY_FIFO_BUFFER_MAX                  UINT8_C(255) // Don't change this
#define BHY_COM_BUFFER_SIZE                  UINT8_C(16)

#define BHY_FIFO_PAGE_SIZE                   BHY_REG_BUFFER_LENGTH
#define ARDUINO_I2C_BUFFER_SIZE              UINT8_C(32)

#define BHY_SENSOR_CONFIGURATION_SIZE        UINT8_C(8)
#define BHY_SENSOR_INFORMATION_SIZE          UINT8_C(16)
#define BHY_PHYSICAL_SENSOR_INFORMATION_SIZE UINT8_C(15)

class BHYSensor {
public:
uint8_t deviceId;
uint8_t productId;
int8_t status;

#ifdef DEBUG_MODE
void setDebug(Print* debug, bhyDebugLevel level);
void setCommDump(bool enable);
void setMethodTrace(bool enable);
void setEventDump(bool enable);

#endif

BHYSensor();

int8_t begin(uint8_t i2cAddress = BHY_I2C_ADDR, TwoWire &wire = Wire);
static String getErrorString(int8_t status);

int8_t read(uint8_t regAddr, uint8_t *data);
int8_t read(uint8_t regAddr, uint8_t *data, uint16_t length);

int8_t write(uint8_t regAddr, uint8_t data);
int8_t write(uint8_t regAddr, uint8_t *data, uint16_t length);

uint8_t readByte(uint8_t regAddr);
uint16_t readShort(uint8_t regAddr);
uint32_t readInteger(uint8_t regAddr);

int8_t writeByte(uint8_t regAddr, uint8_t data);
int8_t writeShort(uint8_t regAddr, uint16_t data);
int8_t writeInteger(uint8_t regAddr, uint32_t data);

int8_t loadFirmware(const uint8_t *bhyFW);

void run(void);

int8_t configVirtualSensor(bhyVirtualSensor sensorId,
                           bool wakeup,
                           bhyFlush flushSensor,
                           uint16_t samplingFreq,
                           uint16_t maxReportLatency,
                           uint16_t changeSensitivity,
                           uint16_t dynamicRange);
int8_t configVirtualSensor(bhyVirtualSensor sensorId,
                           bool wakeup,
                           bhyFlush flushSensor,
                           bhySensorConfiguration configuration);
int8_t disableVirtualSensor(bhyVirtualSensor sensorId, bool wakeup);

static String getSensorName(uint8_t type);
static String getMetaEventName(bhyMetaEventType type);

int8_t getConfiguration(bhyVirtualSensor sensor, bool wakeup, bhySensorConfiguration *configuration);
int8_t getSensorInformation(bhyVirtualSensor sensor, bool wakeup, bhySensorInformation *information);
int8_t getSensorStatus(bhyVirtualSensor sensor, bool wakeup, bhySensorStatus *sensorStatus);

int8_t installSensorCallback(bhyVirtualSensor id, bool wakeup, bhyCallbackFptr callback);
int8_t installSensorCallback(bhyVirtualSensor id, bool wakeup, bhyCallbackFptrTypeOnly callback);
int8_t installSensorCallback(bhyVirtualSensor id, bool wakeup, bhyCallbackFptrU8 callback);
int8_t installSensorCallback(bhyVirtualSensor id, bool wakeup, bhyCallbackFptrU16 callback);
int8_t installSensorCallback(bhyVirtualSensor id, bool wakeup, bhyCallbackFptrActivity callback);
int8_t installSensorCallback(bhyVirtualSensor id, bool wakeup, bhyCallbackFptrFloat callback);
int8_t installSensorCallback(bhyVirtualSensor id, bool wakeup, bhyCallbackFptrBytes callback);
int8_t installSensorCallback(bhyVirtualSensor id, bool wakeup, bhyCallbackFptrQuaternion callback);
int8_t installSensorCallback(bhyVirtualSensor id, bool wakeup, bhyCallbackFptrVector callback);
int8_t installSensorCallback(bhyVirtualSensor id, bool wakeup, bhyCallbackFptrVectorUncalib callback);
int8_t uninstallSensorCallback(bhyVirtualSensor id, bool wakeup);

int8_t installMetaCallback(bhyMetaEventType type, bool wakeup, bhyCallbackFptrMetaEvent callback);
int8_t uninstallMetaCallback(bhyMetaEventType type, bool wakeup);

int8_t installTimeCallback(bool lsw, bhyCallbackFptrTimestamp callback);
int8_t uninstallTimeCallback(bool lsw);

bool isMetaEventEnabled(bhyMetaEventType type, bool wakeup);
bool isMetaInterruptEnabled(bhyMetaEventType type, bool wakeup);
int8_t enableMetaEvent(bhyMetaEventType type, bool wakeup, bool enable);
int8_t enableMetaEventInterrupt(bhyMetaEventType type, bool wakeup, bool enable);

bhyTimestamp getLastTimestamp(void);
bhyTimestamp getLastIrqTimestamp(void);     //Timestamp of last host interrupt
bhyTimestamp getCurrentTime(void);

uint16_t getRamVersion();
uint8_t getProductId();
uint8_t getRevisionId();
uint16_t getRomVersion();

uint16_t getFifoWatermark(bool wakeup);
int8_t setFifoWatermark(bool wakeup, uint16_t bytes);
uint16_t getFifoSize(bool wakeup);
int8_t setFifoFlush(uint8_t value);
uint8_t getFifoFlush();

int8_t getChipControl(uint8_t *value);
int8_t setChipControl(uint8_t value);

int8_t setResetRequest(uint8_t request);

int8_t setHostInterfaceControlBit(uint8_t bit, uint8_t value);
int8_t getHostInterfaceControl(bhyHostInterfaceControl *interfaceControl);

int8_t getHostStatus(bhyHostStatus *hostStatus);
int8_t getInterruptStatus(bhyInterruptStatus *interruptStatus);
int8_t getChipStatus(bhyChipStatus *chipStatus);
int8_t getPhysicalSensorStatus(bhyPhysicalStatus *accel, bhyPhysicalStatus *gyro, bhyPhysicalStatus *mag);

protected:
TwoWire* i2c;
    #ifdef DEBUG_MODE
Print* debugOut = NULL;
bhyDebugLevel debugLevel = BHY_NONE;
bool commDump = false;
bool methodTrace = false;
bool eventDump = false;
#endif
bhySensorStatusBank sensorStatusBank;

uint8_t buffer[BHY_FIFO_BUFFER_SIZE];
uint8_t bufferEnd;
uint8_t bufferStart;
int16_t bufferUsed;
uint16_t bytesWaiting;

#ifdef DEBUG_MODE
void dumpBuffer(uint8_t length);
void dumpBuffer(uint8_t prefix, uint8_t length);

#endif

uint32_t lastTimestamp;

uint8_t nextEventId;
bhyDataType nextEventDataType;
int16_t nextEventDataSize;
bhyVirtualSensor nextEventSensorId;
bool nextEventIsWakeup;

uint8_t frameSizes[17] = {
    BHY_FRAME_SIZE_PADDING, BHY_FRAME_SIZE_QUATERNION, BHY_FRAME_SIZE_VECTOR, BHY_FRAME_SIZE_SCALAR_U8,
    BHY_FRAME_SIZE_SCALAR_U16, BHY_FRAME_SIZE_SCALAR_S16, BHY_FRAME_SIZE_SCALAR_U24, BHY_FRAME_SIZE_SENSOR_EVENT,
    BHY_FRAME_SIZE_UNCALIB_VECTOR, BHY_FRAME_SIZE_META_EVENT, BHY_FRAME_SIZE_BSX, BHY_FRAME_SIZE_DEBUG,
    BHY_FRAME_SIZE_CUS1, BHY_FRAME_SIZE_CUS2, BHY_FRAME_SIZE_CUS3, BHY_FRAME_SIZE_CUS4, BHY_FRAME_SIZE_CUS5, };

uint32_t getBhyCrc();

int8_t updateBuffer();
uint8_t checkForData();
void processEvent(void);

int8_t checkNextEvent();
void clearNextEvent();

void advanceEvent();

uint16_t getBytesWaiting();
uint8_t getNextBufferBytes(uint8_t* copy, int16_t* size);

uint8_t getNextEventId();
bhyDataType getNextEventDataType();
int16_t getNextEventSize();
bhyVirtualSensor getNextEventSensorId();
bool getNextEventIsWakeup();
int8_t skipNextEvent();
void skipBytes(uint8_t number);
void skipBytesUntil(uint8_t value);
void moveBufferCursorBack(uint8_t number);

bhyDataType getDataType(uint8_t sensorId);

void finishCurrentEvent();
void parseBuffer(void);
void parseBufferQuaternion(void);
void parseBufferVector(void);
void parseBufferU8(void);
void parseBufferU16(void);
void parseBufferScaledFloat(void);
void parseBufferU24(void);
void parseBuffer(bhyVectorUncalib *data);
void parseBufferMetaevent(void);
void parseBufferDebugEvent(void);
void parseBuffer(bhyBSX *data);
void parseBufferActivity(void);

uint8_t commBuffer[BHY_COM_BUFFER_SIZE] = { 0, };

int8_t pageSelect(bhyPage page, uint8_t parameter);
int8_t writeParameterPage(bhyPage page, uint8_t parameter);
int8_t writeConfiguration(uint8_t parameter);
int8_t readParameterPage(bhyPage page, uint8_t parameter);

bhyCallbackFptr callbacks[SENSOR_CALLBACK_LIST_NUM + 1];
bhyCallbackFptrMetaEvent metaCallbacks[METAEVENT_CALLBACK_LIST_NUM + 1];
bhyCallbackFptrTimestamp timeCallbacks[TIMESTAMP_CALLBACK_LIST_NUM + 1];

int8_t getSensorStatusBank(uint8_t parameter, uint8_t sensor_type);
int8_t readTimestamps(uint32_t *hostIrq, uint32_t *current);
int8_t getSoftPassThrough(bhySoftPassThrough *data, uint8_t parameter);
int8_t setSoftPassThrough(bhySoftPassThrough *data, uint8_t parameter);

int8_t getParameterPageSelect(uint8_t page_select, uint8_t *parameter_page);
int8_t setParameterPageSelect(uint8_t value);
int8_t getValue(uint8_t *value);
int8_t setValue(uint8_t value);

bhyTimestamp convertTime(uint32_t ticks);

};

#endif /* BHY_H_ */
