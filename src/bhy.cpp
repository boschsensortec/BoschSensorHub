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
 * @file    bhy.cpp
 * @date    30.1.2019
 * @version 0.1.0
 *
 */

#include "bhy.h"
#include "bhy_defs_internal.h"

#ifndef ARDUINO_I2C_BUFFER_SIZE
#define ARDUINO_I2C_BUFFER_SIZE UINT8_C(32)
#endif

BHYSensor::BHYSensor()
{
    for (uint8_t i = 0; i < SENSOR_CALLBACK_LIST_NUM + 1; i++)
        callbacks[i].invalid = NULL;
    deviceId = BHY_I2C_ADDR;
    i2c = NULL;
    productId = 0;
    bytesWaiting = 0;
    bufferEnd = 0;
    bufferStart = 0;
    bufferUsed = 0;
    lastTimestamp = 0;
    nextEventId = BHY_SID_NONE;
    nextEventDataType = BHY_INVALID_DATA_TYPE;
    nextEventDataSize = 0;
    nextEventSensorId = BHY_VS_INVALID;
    nextEventIsWakeup = false;
    for (uint32_t j = 0; j < BHY_FIFO_BUFFER_SIZE; j++)
        buffer[j] = 0;
    status = BHY_OK;
}

int8_t BHYSensor::begin(uint8_t i2cAddress, TwoWire &wire)
{
    deviceId = i2cAddress;
    i2c = &wire;
    const uint8_t nTries = 5;
    uint8_t tries = nTries;
    uint8_t data;

    i2c->begin();

    while (tries > 0)
    {
        status = read(BHY_REG_PRODUCT_ID_ADDR, &data);
        --tries;

        if (data == BHY_PRODUCT_ID)
            break;
#ifdef DEBUG_MODE
        if (debugOut && (debugLevel >= BHY_INFORMATIVE))
        {
            debugOut->print("\tProduct ID read #");
            debugOut->print(nTries - tries);
            debugOut->print(": ");
            if (data < 16)
                debugOut->print("0");
            debugOut->print(data, HEX);
            debugOut->print(" [read return code ");
            debugOut->print(status);
            debugOut->println("]");
        }
#endif

        delay(BHY_ACK_DELAY);
    }

    if (data != BHY_PRODUCT_ID)
        return status = BHY_E_PRODUCT_ID_MISMATCH;

    if (status < 0)
        return status;

    productId = data;

    // Request reset
    status = write(BHY_REG_RESET_REQUEST_ADDR, BHY_ENABLE);

    if (status < 0)
        return status;

#ifdef DEBUG_MODE
    if (debugOut && (debugLevel >= BHY_DEBUG) && methodTrace)
    {
        debugOut->println("/begin()");
    }
#endif

    return status;
}

int8_t BHYSensor::loadFirmware(const uint8_t *bhyFW)
{
#ifdef DEBUG_MODE
    if (debugOut && (debugLevel >= BHY_DEBUG) && methodTrace)
    {
        debugOut->print("loadFirmware(0x");
        debugOut->print((long)bhyFW, HEX);
        debugOut->println(")");
    }
#endif

    uint8_t fwHeader[BHY_SIGNATURE_MEM_LEN] = { 0, };

    for (uint8_t i = 0; i < BHY_SIGNATURE_MEM_LEN; ++i)
    {
        fwHeader[i] = bhyFW[i];
    }

    // Verify the signature
    if ((fwHeader[BHY_SIGNATURE_1] != BHY_IMAGE_SIGNATURE1) || (fwHeader[BHY_SIGNATURE_2] != BHY_IMAGE_SIGNATURE2))
    {
        return status = BHY_E_INVALID_FIRMWARE;
    }

    // Verify the ROM and RAM versions
    uint16_t signatureFlag = fwHeader[BHY_SIG_FLAG_1_POS] + ((uint16_t)fwHeader[BHY_SIG_FLAG_2_POS] << 8);
    uint8_t romVerExp = (signatureFlag >> 11) & 0x03;

    uint16_t romVersion = getRomVersion();

#ifdef DEBUG_MODE
    if (debugOut && (debugLevel >= BHY_INFORMATIVE))
    {
        debugOut->print("Firmware Signature Flag: 0x");
        debugOut->print(signatureFlag, HEX);
        debugOut->print(", Expected ROM Version: 0x");
        debugOut->println(romVerExp, HEX);

        debugOut->print("Read ROM Version: 0x");
        debugOut->print(romVersion, HEX);
        debugOut->print(" [read return code ");
        debugOut->print(status);
        debugOut->println("]");
    }
#endif

    if (!(romVerExp == BHY_ROM_VER_DI01 && romVersion == BHY_ROM_VERSION_DI01) &&
        !(romVerExp == BHY_ROM_VER_DI03 && romVersion == BHY_ROM_VERSION_DI03))
    {
        return status = BHY_E_RAMPATCH_MISMATCH;
    }

    // Remove the first 16 bytes
    uint32_t fwLength = 16 + bhyFW[12] + ((uint32_t)bhyFW[13] << 8);
    uint32_t dataToProcess = fwLength - (BHY_SIGNATURE_MEM_LEN - 1);

#ifdef DEBUG_MODE
    if (debugOut && (debugLevel >= BHY_INFORMATIVE))
    {
        debugOut->print("Binary to load size: ");
        debugOut->println(dataToProcess);
    }
#endif

    // Request reset
    status = write(BHY_REG_RESET_REQUEST_ADDR, BHY_ENABLE);

    if (status < 0)
        return status;

    status = write(BHY_REG_CHIP_CONTROL_ADDR, BHY_CHIP_CTRL_HOST_UPLOAD_BIT);

    if (status < 0)
        return status;

    status = write(BHY_REG_UPLOAD_0_ADDR, 0x00);

    if (status < 0)
        return status;

    status = write(BHY_REG_UPLOAD_1_ADDR, 0x00);

    if (status < 0)
        return status;

    uint32_t nLoops = dataToProcess / BHY_RAM_WRITE_LENGTH_API;

    uint8_t fwSnippet[BHY_RAM_WRITE_LENGTH_API] = { 0, };
    uint32_t offset = (BHY_SIGNATURE_MEM_LEN - 1);
    uint32_t bytesToWrite;
    int tries = 5;
    status = BHY_ERROR;

    // If a successful status was received after the load of firmware, break or else if tries run out
    while ((tries-- > 0) && (status != 0))
    {
        offset = (BHY_SIGNATURE_MEM_LEN - 1);
        for (uint16_t i = 0; i <= nLoops; ++i)
        {
            bytesToWrite =
                ((i ==
                  nLoops) ? (dataToProcess % BHY_RAM_WRITE_LENGTH_API) : BHY_RAM_WRITE_LENGTH_API) /
                BHY_RAM_WRITE_LENGTH;

            // Reverse the data
            // 32bit processor (4 bytes) and endianess of the FW to be changed before upload.
            for (uint32_t j = 1; j <= bytesToWrite; ++j)
            {
                for (uint32_t k = 0; k < BHY_RAM_WRITE_LENGTH; ++k)
                {
                    fwSnippet[k +
                              ((j - 1) * BHY_RAM_WRITE_LENGTH)] = bhyFW[offset + BHY_RAM_WRITE_LENGTH * j - (k + 1)];
                }
            }

            if (bytesToWrite != 0)
            {
                status = write(BHY_REG_UPLOAD_DATA_ADDR, fwSnippet, bytesToWrite * BHY_RAM_WRITE_LENGTH);
                if (status < 0)
                    return status;
            }

            offset += (bytesToWrite * BHY_RAM_WRITE_LENGTH);
        }
    }

    // Read the CRC data from the FW
    uint32_t crcFromFw =
        (((((((uint32_t)fwHeader[BHY_CRC_HOST_FILE_LSB + 3] << 8) | fwHeader[BHY_CRC_HOST_FILE_LSB + 2]) << 8) |
           fwHeader[BHY_CRC_HOST_FILE_LSB + 1]) << 8) | fwHeader[BHY_CRC_HOST_FILE_LSB]);

    uint32_t crcFromDevice = getBhyCrc();

#ifdef DEBUG_MODE
    if (debugOut && (debugLevel >= BHY_INFORMATIVE))
    {
        debugOut->print("Expected CRC: 0x");
        debugOut->println(crcFromFw, HEX);
        debugOut->print("Device CRC: 0x");
        debugOut->println(crcFromDevice, HEX);
    }
#endif

    if (status < 0)
        return status;

    if (crcFromFw != crcFromDevice)
    {
#ifdef DEBUG_MODE
        if (debugOut && (debugLevel >= BHY_ERROR) && (debugLevel < BHY_INFORMATIVE))
        {
            debugOut->print("Expected CRC: 0x");
            debugOut->println(crcFromFw, HEX);
            debugOut->print("Host CRC: 0x");
            debugOut->println(crcFromDevice, HEX);
        }
#endif

        return status = BHY_E_CRC_MISMATCH;
    }

    status = write(BHY_REG_CHIP_CONTROL_ADDR, BHY_CHIP_CTRL_CPU_RUN_BIT); //

#ifdef DEBUG_MODE
    if (debugOut && (debugLevel >= BHY_DEBUG) && methodTrace)
    {
        debugOut->print("/loadFirmware()");
    }
#endif

    return status;
}

bhyTimestamp BHYSensor::getLastIrqTimestamp(void)
{
    return convertTime(readInteger(BHY_IRQ_TIMESTAMP_ADDR));
}

bhyTimestamp BHYSensor::getCurrentTime(void)
{
    status = readParameterPage(BHY_SYSTEM_PAGE, BHY_HOST_IRQ_TIMESTAMP);

    if (status != BHY_OK)
        return convertTime(0);

    uint32_t ticks =
        (((((((uint32_t)commBuffer[3] << 24) | commBuffer[2]) << 16) | commBuffer[1]) << 8) | commBuffer[0]);

    return convertTime(ticks);
}

uint16_t BHYSensor::getRomVersion()
{
    return readShort(BHY_ROM_VERSION_ADDR);
}

uint16_t BHYSensor::getRamVersion()
{
    return readShort(BHY_RAM_VERSION_ADDR);
}

uint8_t BHYSensor::getProductId()
{
    return readByte(BHY_REG_PRODUCT_ID_ADDR);
}

uint8_t BHYSensor::getRevisionId()
{
    return readByte(BHY_REG_REVISION_ID_ADDR);
}

uint32_t BHYSensor::getBhyCrc()
{
    return readInteger(BHY_REG_CRC_HOST_ADDR);
}

uint8_t BHYSensor::getFifoFlush()
{
    return readByte(BHY_REG_FIFO_FLUSH_ADDR);
}

int8_t BHYSensor::setFifoFlush(uint8_t value)
{
    return status = write(BHY_REG_FIFO_FLUSH_ADDR, value);
}

int8_t BHYSensor::getChipControl(uint8_t *value)
{
    return status = read(BHY_REG_CHIP_CONTROL_ADDR, value);
}

int8_t BHYSensor::setChipControl(uint8_t value)
{
    return status = write(BHY_REG_CHIP_CONTROL_ADDR, value);
}

int8_t BHYSensor::getHostStatus(bhyHostStatus *hostStatus)
{
    uint8_t data = 0;

    status = read(BHY_REG_HOST_STATUS_ADDR, &hostStatus->value);

    if (status != BHY_OK)
        return status;

    return status;
}

int8_t BHYSensor::getInterruptStatus(bhyInterruptStatus *interruptStatus)
{
    status = read(BHY_REG_INT_STATUS_ADDR, &interruptStatus->value);

    return status;
}

int8_t BHYSensor::getChipStatus(bhyChipStatus *chipStatus)
{
    status = read(BHY_REG_CHIP_STATUS_ADDR, &chipStatus->value);

    return status;
}

int8_t BHYSensor::setHostInterfaceControlBit(uint8_t bit, uint8_t value)
{
    if (bit >= 8)
        return status = BHY_E_OUT_OF_RANGE;

    uint8_t data = 0;

    status = read(BHY_REG_HOST_INTERFACE_CONTROL_ADDR, &data);

    if (status != BHY_OK)
        return status;

    data = (data & ~(1 << bit)) | ((value & 1) << bit);

    return status = write(BHY_REG_HOST_INTERFACE_CONTROL_ADDR, data);
}

int8_t BHYSensor::setResetRequest(uint8_t request)
{
    return status = write(BHY_REG_RESET_REQUEST_ADDR, request);
}

bool BHYSensor::isMetaEventEnabled(bhyMetaEventType type, bool wakeup)
{
    uint8_t effectiveType = type;

    if (type > 32)
    {
        status = BHY_E_OUT_OF_RANGE;

        return false;
    }

    status = readParameterPage(BHY_SYSTEM_PAGE, (wakeup) ? BHY_META_EVENT_CONTROL_WAKEUP : BHY_META_EVENT_CONTROL);

    if (status != BHY_OK)
        return false;

    uint8_t shift = effectiveType * 2 - 1;

    uint8_t bit = shift % 8;
    uint8_t byte = shift / 8;

    return (commBuffer[byte] >> bit) & 1;
}

bool BHYSensor::isMetaInterruptEnabled(bhyMetaEventType type, bool wakeup)
{
    uint8_t effectiveType = type;

    if (type > 32)
    {
        status = BHY_E_OUT_OF_RANGE;

        return false;
    }

    status = readParameterPage(BHY_SYSTEM_PAGE, (wakeup) ? BHY_META_EVENT_CONTROL_WAKEUP : BHY_META_EVENT_CONTROL);

    if (status != BHY_OK)
        return false;

    uint8_t shift = (effectiveType - 1) * 2;

    uint8_t bit = shift % 8;
    uint8_t byte = shift / 8;

    return (commBuffer[byte] >> bit) & 1;
}

int8_t BHYSensor::enableMetaEvent(bhyMetaEventType type, bool wakeup, bool enable)
{
    uint8_t effectiveType = type;

    if (type > 32)
    {
        status = BHY_E_OUT_OF_RANGE;

        return false;
    }

    status = readParameterPage(BHY_SYSTEM_PAGE, (wakeup) ? BHY_META_EVENT_CONTROL_WAKEUP : BHY_META_EVENT_CONTROL);

    if (status != BHY_OK)
        return status;

    uint8_t shift = effectiveType * 2 - 1;

    uint8_t bit = shift % 8;
    uint8_t byte = shift / 8;

    commBuffer[byte] = (commBuffer[byte] & ~(1 << bit)) | ((uint8_t)enable << bit);

    status = writeParameterPage(BHY_SYSTEM_PAGE,
                                0x80 + (wakeup) ? BHY_META_EVENT_CONTROL_WAKEUP : BHY_META_EVENT_CONTROL);

    return status;
}

int8_t BHYSensor::enableMetaEventInterrupt(bhyMetaEventType type, bool wakeup, bool enable)
{
    uint8_t effectiveType = type;

    if (type > 32)
    {
        status = BHY_E_OUT_OF_RANGE;

        return false;
    }

    status = readParameterPage(BHY_SYSTEM_PAGE, (wakeup) ? BHY_META_EVENT_CONTROL_WAKEUP : BHY_META_EVENT_CONTROL);

    if (status != BHY_OK)
        return status;

    uint8_t shift = (effectiveType - 1) * 2;

    uint8_t bit = shift % 8;
    uint8_t byte = shift / 8;

    commBuffer[byte] = (commBuffer[byte] & ~(1 << bit)) | ((uint8_t)enable << bit);

    status = writeParameterPage(BHY_SYSTEM_PAGE,
                                0x80 + (wakeup) ? BHY_META_EVENT_CONTROL_WAKEUP : BHY_META_EVENT_CONTROL);

    return status;
}

#ifdef DEBUG_MODE
void BHYSensor::setDebug(Print* debug, bhyDebugLevel level)
{
    debugOut = debug;
    debugLevel = level;
}

void BHYSensor::setCommDump(bool enable)
{
    commDump = enable;
}

void BHYSensor::setMethodTrace(bool enable)
{
    methodTrace = enable;
}

void BHYSensor::setEventDump(bool enable)
{
    eventDump = enable;
}

void BHYSensor::dumpBuffer(uint8_t length)
{
    if (!debugOut)
        return;

    for (uint8_t i = 0; i < length; ++i)
    {
        debugOut->print(" ");
        if (buffer[bufferStart + i] < 16)
            debugOut->print("0");
        debugOut->print(buffer[bufferStart + i], HEX);
        if (i % 8 == 7 || i == length - 1)
            debugOut->println("");
    }
}

void BHYSensor::dumpBuffer(uint8_t prefix, uint8_t length)
{
    if (!debugOut)
        return;

    debugOut->print(" ");
    if (prefix < 16)
        debugOut->print("0");
    debugOut->print(prefix, HEX);

    for (uint8_t i = 1; i <= length; ++i)
    {
        debugOut->print(" ");
        if (buffer[bufferStart + i - 1] < 16)
            debugOut->print("0");
        debugOut->print(buffer[bufferStart + i - 1], HEX);
        if (i % 8 == 7 || i == length)
            debugOut->println("");
    }
}
#endif

int8_t BHYSensor::updateBuffer()
{
#ifdef DEBUG_MODE
    if (debugOut && (debugLevel >= BHY_DEBUG) && methodTrace)
    {
        debugOut->println("updateBuffer()");
    }

#endif
    if (bytesWaiting == 0)
    {
        checkForData();
        if (bytesWaiting == 0)
        {
#ifdef DEBUG_MODE
            if (debugOut && (debugLevel >= BHY_DEBUG))
            {
                debugOut->println("\tNothing to update.");
                if (methodTrace)
                    debugOut->println("/updateBuffer()");
            }
#endif

            return status;
        }
    }

    int16_t bufferCapacity = BHY_FIFO_BUFFER_SIZE - bufferUsed; // Bad if BHY_FIFO_BUFFER_SIZE != 256

#ifdef DEBUG_MODE
    if (debugOut && (debugLevel >= BHY_INFORMATIVE))
    {
        debugOut->print("\tCurrent available buffer capacity: ");
        debugOut->print(bufferCapacity);
    }
#endif

    if (bufferCapacity == 0)
    {
#ifdef DEBUG_MODE
        if (debugOut && (debugLevel >= BHY_DEBUG))
        {
            debugOut->println("\tNo space in buffer.");
            if (methodTrace)
                debugOut->println("/updateBuffer()");
        }
#endif

        return status;
    }

    int16_t toRead = 0;

    if (bytesWaiting > bufferCapacity)
    {
        // Keep the buffer aligned with the internal buffer of the sensor FIFO, of size BHY_FIFO_PAGE_SIZE
        toRead = bufferCapacity - (bufferCapacity % BHY_FIFO_PAGE_SIZE);
    }
    else
    {
        toRead = bytesWaiting;
    }

    if (toRead == 0)
    {
#ifdef DEBUG_MODE
        if (debugOut && (debugLevel >= BHY_DEBUG))
        {
            debugOut->println("\tNo read possible.");
            if (methodTrace)
                debugOut->println("/updateBuffer()");
        }
#endif

        return status;
    }

#ifdef DEBUG_MODE
    if (debugOut && (debugLevel >= BHY_INFORMATIVE))
    {
        debugOut->print("\tNumber of bytes to read: ");
        debugOut->println(toRead);
    }
#endif

    uint8_t thisRead = 0;
    uint8_t reg = BHY_REG_BUFFER_ZERO_ADDR;
    uint16_t bufferSpace;

    while (toRead > 0)
    {
        if (bufferEnd >= bufferStart)
            bufferSpace = BHY_FIFO_BUFFER_SIZE - bufferEnd; // Bad if BHY_FIFO_BUFFER_SIZE != 256
        else
            bufferSpace = bufferStart - bufferEnd; // Bad if BHY_FIFO_BUFFER_SIZE != 256

#ifdef DEBUG_MODE
        if (debugOut && (debugLevel >= BHY_INFORMATIVE))
        {
            debugOut->print("\tBuffer space available for this read: ");
            debugOut->println(bufferSpace);
        }
#endif

        // Ensure we don't overflow the pseudo-circular buffer
        thisRead = (ARDUINO_I2C_BUFFER_SIZE < bufferSpace) ? ARDUINO_I2C_BUFFER_SIZE : bufferSpace; // minimum

        if (toRead < thisRead)
            thisRead = toRead;

#ifdef DEBUG_MODE
        if (debugOut && (debugLevel >= BHY_INFORMATIVE))
        {
            debugOut->print("\tNumber of bytes in this transaction: ");
            debugOut->println(thisRead);
        }
#endif

        read(reg, &buffer[bufferEnd], thisRead);

        reg = (reg + thisRead) % BHY_REG_BUFFER_LENGTH; // Register address to begin next read on;

        bufferEnd += thisRead; // Bad if BHY_FIFO_BUFFER_SIZE != 256
        bufferUsed += thisRead;

        toRead -= thisRead;
        bytesWaiting -= thisRead;

#ifdef DEBUG_MODE
        if (debugOut && (debugLevel >= BHY_INTERNAL))
        { //INTERNAL
            debugOut->print("\tBuffer End: ");
            debugOut->print(bufferEnd);
            debugOut->print(", Buffer Used: ");
            debugOut->print(bufferUsed);
            debugOut->print(thisRead);
            debugOut->print(", thisRead: ");
            debugOut->print(thisRead);
            debugOut->print(", Next register: ");
            debugOut->println(reg);
        }
#endif
    }

    if (bufferStart == bufferEnd && bufferUsed == 0)
        bufferStart = bufferEnd = 0; // !!!

#ifdef DEBUG_MODE
    if (debugOut && (debugLevel >= BHY_DEBUG) && methodTrace)
    {
        debugOut->println("/updateBuffer()");
    }
#endif

    checkNextEvent();

    return status;
}

uint8_t BHYSensor::checkForData()
{
    bytesWaiting = readShort(BHY_REG_BYTES_REMAINING_ADDR);

#ifdef DEBUG_MODE
    if (debugOut && (debugLevel >= BHY_INFORMATIVE))
    {
        debugOut->print("\tBytes waiting in sensor FIFO: ");
        debugOut->println(bytesWaiting);
    }
#endif

    return status;
}

uint16_t BHYSensor::getBytesWaiting()
{
    return bytesWaiting;
}

uint8_t BHYSensor::getNextBufferBytes(uint8_t* copy, int16_t* size)
{
    *size = (*size < bufferUsed) ? *size : bufferUsed;

    for (int16_t i = 0; i < *size; ++i)
    {
        copy[i] = buffer[bufferStart + i];
    }

    return status;
}

bhyDataType BHYSensor::getDataType(uint8_t sensorId)
{
    bhyDataType newEventDataType;

    switch (sensorId)
    {
    case BHY_SID_PADDING:
        newEventDataType = BHY_PADDING_TYPE;
        break;

    case BHY_SID_SIGNIFICANT_MOTION:
    case BHY_SID_SIGNIFICANT_MOTION_WAKEUP:
    case BHY_SID_STEP_DETECTOR:
    case BHY_SID_STEP_DETECTOR_WAKEUP:
    case BHY_SID_TILT_DETECTOR:
    case BHY_SID_TILT_DETECTOR_WAKEUP:
    case BHY_SID_WAKE_GESTURE:
    case BHY_SID_WAKE_GESTURE_WAKEUP:
    case BHY_SID_GLANCE_GESTURE:
    case BHY_SID_GLANCE_GESTURE_WAKEUP:
    case BHY_SID_PICKUP_GESTURE:
    case BHY_SID_PICKUP_GESTURE_WAKEUP:
        newEventDataType = BHY_SENSOR_EVENT_TYPE;
        break;

    case BHY_SID_ROTATION_VECTOR:
    case BHY_SID_ROTATION_VECTOR_WAKEUP:
    case BHY_SID_GAME_ROTATION_VECTOR:
    case BHY_SID_GAME_ROTATION_VECTOR_WAKEUP:
    case BHY_SID_GEOMAGNETIC_ROTATION_VECTOR:
    case BHY_SID_GEOMAGNETIC_ROTATION_VECTOR_WAKEUP:
        newEventDataType = BHY_QUATERNION_TYPE;
        break;

    case BHY_SID_ACCELEROMETER:
    case BHY_SID_ACCELEROMETER_WAKEUP:
    case BHY_SID_MAGNETOMETER:
    case BHY_SID_MAGNETOMETER_WAKEUP:
    case BHY_SID_ORIENTATION:
    case BHY_SID_ORIENTATION_WAKEUP:
    case BHY_SID_GYROSCOPE:
    case BHY_SID_GYROSCOPE_WAKEUP:
    case BHY_SID_GRAVITY:
    case BHY_SID_GRAVITY_WAKEUP:
    case BHY_SID_LINEAR_ACCELERATION:
    case BHY_SID_LINEAR_ACCELERATION_WAKEUP:
        newEventDataType = BHY_VECTOR_TYPE;
        break;

    case BHY_SID_HEART_RATE:
    case BHY_SID_HEART_RATE_WAKEUP:
        newEventDataType = BHY_SCALAR_TYPE_U8;
        break;

    case BHY_SID_LIGHT:
    case BHY_SID_LIGHT_WAKEUP:
    case BHY_SID_PROXIMITY:
    case BHY_SID_PROXIMITY_WAKEUP:
    case BHY_SID_HUMIDITY:
    case BHY_SID_HUMIDITY_WAKEUP:
    case BHY_SID_STEP_COUNTER:
    case BHY_SID_STEP_COUNTER_WAKEUP:
    case BHY_SID_TIMESTAMP_LSW:
    case BHY_SID_TIMESTAMP_LSW_WAKEUP:
    case BHY_SID_TIMESTAMP_MSW:
    case BHY_SID_TIMESTAMP_MSW_WAKEUP:
        newEventDataType = BHY_SCALAR_TYPE_U16;
        break;

    case BHY_SID_TEMPERATURE:
    case BHY_SID_TEMPERATURE_WAKEUP:
    case BHY_SID_AMBIENT_TEMPERATURE:
    case BHY_SID_AMBIENT_TEMPERATURE_WAKEUP:
        newEventDataType = BHY_SCALAR_TYPE_S16;
        break;

    case BHY_SID_BAROMETER:
    case BHY_SID_BAROMETER_WAKEUP:
        newEventDataType = BHY_SCALAR_TYPE_U24;
        break;

    case BHY_SID_UNCALIBRATED_MAGNETOMETER:
    case BHY_SID_UNCALIBRATED_MAGNETOMETER_WAKEUP:
    case BHY_SID_UNCALIBRATED_GYROSCOPE:
    case BHY_SID_UNCALIBRATED_GYROSCOPE_WAKEUP:
        newEventDataType = BHY_VECTOR_TYPE_UNCALIBRATED;
        break;

    case BHY_SID_META_EVENT:
    case BHY_SID_META_EVENT_WAKEUP:
        newEventDataType = BHY_META_TYPE;
        break;

    case BHY_SID_DEBUG:
        newEventDataType = BHY_DEBUG_TYPE;
        break;

    case BHY_SID_BSX_C:
    case BHY_SID_BSX_B:
    case BHY_SID_BSX_A:
        newEventDataType = BHY_BSX_TYPE;
        break;

    case BHY_SID_CUS1:
    case BHY_SID_CUS1_WAKEUP:
        newEventDataType = BHY_CUSTOM_TYPE_1;
        break;

    case BHY_SID_CUS2:
    case BHY_SID_CUS2_WAKEUP:
        newEventDataType = BHY_CUSTOM_TYPE_2;
        break;

    case BHY_SID_CUS3:
    case BHY_SID_CUS3_WAKEUP:
        newEventDataType = BHY_CUSTOM_TYPE_3;
        break;

    case BHY_SID_CUS4:
    case BHY_SID_CUS4_WAKEUP:
        newEventDataType = BHY_CUSTOM_TYPE_4;
        break;

    case BHY_SID_CUS5:
    case BHY_SID_CUS5_WAKEUP:
        newEventDataType = BHY_CUSTOM_TYPE_5;
        break;

    // TODO PDR

    case BHY_SID_ACTIVITY:
    case BHY_SID_ACTIVITY_WAKEUP:
        newEventDataType = BHY_SCALAR_TYPE_S16;     //ACTIVITY_TYPE;
        break;

    // the VS sensor ID is unknown. Either the sync has been lost or the
    // ram patch implements a new sensor ID that this driver doesn't yet
    // support
    default:
        newEventDataType = BHY_INVALID_DATA_TYPE;
    }

    return newEventDataType;
}

int8_t BHYSensor::checkNextEvent()
{
#ifdef DEBUG_MODE
    if (debugOut && (debugLevel >= BHY_DEBUG))
    {
        debugOut->print("\tBuffer State: i= ");
        debugOut->print(bufferStart);
        debugOut->print(" ... ");
        debugOut->print(bufferEnd);
        debugOut->print("; using ");
        debugOut->println(bufferUsed);
    }
#endif

    if (nextEventId != BHY_SID_NONE)
        return status;

    if (bufferUsed == 0)
    {
        if ((bufferStart == bufferEnd) && (bufferStart != 0))
        {
#ifdef DEBUG_MODE
            if (debugOut && (debugLevel >= BHY_DEBUG))
            {
                debugOut->println("\tBuffer emptied.");
            }
#endif
            bufferStart = bufferEnd = 0;
        }
        else if (bufferStart != bufferEnd)
        {
#ifdef DEBUG_MODE
            if (debugOut && (debugLevel >= BHY_ERROR))
            {
                debugOut->println("Buffer tracking error.");
            }
#endif
            bufferStart = bufferEnd = 0;
        }

        return status;
    }

    if (bufferUsed <= 0)
    {
#ifdef DEBUG_MODE
        if (debugOut && (debugLevel >= BHY_ERROR))
        {
            debugOut->print("Buffer error, capacity used: ");
            debugOut->print(bufferUsed);
            debugOut->println("; Reseting...");
        }
#endif

        clearNextEvent();

        bufferStart = bufferEnd = 0;
        bufferUsed = 0;

        return status;
    }

    uint8_t newEventId = buffer[bufferStart];
    bhyDataType newEventDataType;
    int16_t _newEventDataSize;
    bhyVirtualSensor newEventSensorId = BHY_VS_INVALID;
    bool newEventIsWakeup = false;

    newEventDataType = getDataType(newEventId);

    if (newEventDataType == BHY_INVALID_DATA_TYPE)
    {
        clearNextEvent();
        nextEventId = newEventId;
        nextEventDataSize = -1;

#ifdef DEBUG_MODE
        if (debugOut && (debugLevel >= BHY_ERROR))
        {
            debugOut->print("Next event has invalid ID: ");
            debugOut->println(newEventId);
        }
#endif
    }
    else if (bufferUsed < frameSizes[newEventDataType])
    {
        clearNextEvent();

#ifdef DEBUG_MODE
        if (debugOut && (debugLevel >= BHY_DEBUG))
        {
            debugOut->print("\tBuffer contains incomplete entry of type: ");
            debugOut->println(newEventId);
        }
#endif
    }
    else
    {
#ifdef DEBUG_MODE
        if (debugOut && (debugLevel >= BHY_INTERNAL))
        {
            debugOut->print("\tNext buffer entry valid.");
        }
#endif

        if (newEventId >= BHY_SID_ACCELEROMETER && newEventId < BHY_SID_WAKEUP_OFFSET)
        {
            newEventSensorId = (bhyVirtualSensor)newEventId;
        }
        else if (newEventId >= (BHY_SID_ACCELEROMETER + BHY_SID_WAKEUP_OFFSET) &&
                 newEventId <= (BHY_SID_ACTIVITY + BHY_SID_WAKEUP_OFFSET))
        {
            newEventSensorId = (bhyVirtualSensor)(newEventId - BHY_SID_WAKEUP_OFFSET);
            newEventIsWakeup = true;
        }
        else if (newEventId == BHY_SID_TIMESTAMP_LSW_WAKEUP || newEventId == BHY_SID_TIMESTAMP_MSW_WAKEUP ||
                 newEventId == BHY_SID_META_EVENT_WAKEUP)
        {
            newEventIsWakeup = true;
        }

        nextEventId = newEventId;
        nextEventDataType = newEventDataType;
        nextEventDataSize = frameSizes[nextEventDataType] - 1;
        nextEventSensorId = newEventSensorId;
        nextEventIsWakeup = newEventIsWakeup;

        bufferStart++; //Advance to data portion
        bufferUsed--;
    }

    return status;
}

uint8_t BHYSensor::getNextEventId()
{
    if (nextEventId != BHY_SID_NONE)
        return nextEventId;

    checkNextEvent();

    return nextEventId;
}

bhyDataType BHYSensor::getNextEventDataType()
{
    if (nextEventId != BHY_SID_NONE)
        return nextEventDataType;

    checkNextEvent();

    return nextEventDataType;
}

int16_t BHYSensor::getNextEventSize()
{
    if (nextEventId != BHY_SID_NONE)
        return nextEventDataSize;

    checkNextEvent();

    return nextEventDataSize;
}

bhyVirtualSensor BHYSensor::getNextEventSensorId()
{
    if (nextEventId != BHY_SID_NONE)
        return nextEventSensorId;

    checkNextEvent();

    return nextEventSensorId;
}

bool BHYSensor::getNextEventIsWakeup()
{
    if (nextEventId != BHY_SID_NONE)
        return nextEventIsWakeup;

    checkNextEvent();

    return nextEventIsWakeup;
}

void BHYSensor::clearNextEvent()
{
    nextEventId = BHY_SID_NONE;
    nextEventDataType = BHY_INVALID_DATA_TYPE;
    nextEventDataSize = 0;
    nextEventSensorId = BHY_VS_INVALID;
    nextEventIsWakeup = false;
}

void BHYSensor::advanceEvent()
{

    nextEventId = BHY_SID_NONE;
    nextEventDataType = BHY_INVALID_DATA_TYPE;
    nextEventDataSize = 0;
    nextEventSensorId = BHY_VS_INVALID;
    nextEventIsWakeup = false;
}

int8_t BHYSensor::skipNextEvent()
{
    if (nextEventId == BHY_SID_NONE)
        return status;
    if (nextEventDataType == BHY_INVALID_DATA_TYPE)
        return status;

    bufferUsed -= nextEventDataSize;
    bufferStart += nextEventDataSize;

    advanceEvent();

    checkNextEvent();

    return status;
}

void BHYSensor::finishCurrentEvent()
{
    if (nextEventId == BHY_SID_NONE)
        return;
    if (nextEventDataType == BHY_INVALID_DATA_TYPE)
        return;

    bufferUsed -= nextEventDataSize;

    advanceEvent();

    checkNextEvent();
}

void BHYSensor::skipBytes(uint8_t number)
{
    uint8_t toSkip = (number < bufferUsed) ? number : bufferUsed;

#ifdef DEBUG_MODE
    if (debugOut && (debugLevel >= BHY_DEBUG))
    {
        debugOut->print("Skipping ");
        debugOut->print(toSkip);
        debugOut->println(" bytes.");
    }
#endif

    bufferUsed -= toSkip;
    bufferStart += toSkip;

    clearNextEvent();

    checkNextEvent();
}

void BHYSensor::skipBytesUntil(uint8_t value)
{
    uint8_t skipped = 0;

    while (bufferUsed && buffer[bufferStart] != value)
    {
        ++bufferStart;
        --bufferUsed;
        ++skipped;
    }

#ifdef DEBUG_MODE
    if (debugOut && (debugLevel >= BHY_DEBUG))
    {
        debugOut->print("Skipped ");
        debugOut->print(skipped);
        debugOut->println(" bytes.");
    }
#endif

    clearNextEvent();

    checkNextEvent();
}

void BHYSensor::moveBufferCursorBack(uint8_t number)
{
    uint8_t toMove = (bufferUsed + number < BHY_FIFO_BUFFER_SIZE) ? number : BHY_FIFO_BUFFER_MAX - bufferUsed;

#ifdef DEBUG_MODE
    if (debugOut && (debugLevel >= BHY_DEBUG))
    {
        debugOut->print("Retracting ");
        debugOut->print(toMove);
        debugOut->println(" bytes.");
    }
#endif

    bufferUsed += toMove;
    bufferStart -= toMove;

    clearNextEvent();

    checkNextEvent();
}

void BHYSensor::run(void)
{
    do
    {
        updateBuffer();
    } while (bytesWaiting);

    do
    {
        processEvent();
    } while (nextEventDataType != BHY_INVALID_DATA_TYPE);
}

void BHYSensor::processEvent(void)
{
    switch (nextEventDataType)
    {
    case BHY_PADDING_TYPE:
        skipNextEvent();
        break;

    case BHY_SENSOR_EVENT_TYPE:
        parseBuffer();
        break;

    case BHY_QUATERNION_TYPE:
        parseBufferQuaternion();
        break;

    case BHY_VECTOR_TYPE:
        parseBufferVector();
        break;

    case BHY_SCALAR_TYPE_U8:
        parseBufferU8();
        break;

    case BHY_SCALAR_TYPE_U16:
        if ((nextEventSensorId == BHY_VS_LIGHT) || (nextEventSensorId == BHY_VS_PROXIMITY) ||
            (nextEventSensorId == BHY_VS_RELATIVE_HUMIDITY))
            parseBufferScaledFloat();
        else
            parseBufferU16();
        break;

    case BHY_SCALAR_TYPE_S16:
        if (nextEventSensorId == BHY_VS_ACTIVITY_RECOGNITION)
            parseBufferActivity();
        else
            parseBufferScaledFloat();
        break;

    case BHY_SCALAR_TYPE_U24:
        parseBufferU24();
        break;

    case BHY_VECTOR_TYPE_UNCALIBRATED:
        skipNextEvent();
        break;

    case BHY_META_TYPE:
        parseBufferMetaevent();
        break;

    case BHY_DEBUG_TYPE:
        parseBufferDebugEvent();
        break;

    case BHY_BSX_TYPE:
        skipNextEvent();
        break;

    case BHY_CUSTOM_TYPE_1:
    case BHY_CUSTOM_TYPE_2:
    case BHY_CUSTOM_TYPE_3:
    case BHY_CUSTOM_TYPE_4:
    case BHY_CUSTOM_TYPE_5:
        skipNextEvent();
        break;

    default:
        if (BHY_INVALID_DATA_TYPE)
            skipBytesUntil(0xFC);
        else
            skipNextEvent();
        break;
    }
}

bhyTimestamp BHYSensor::getLastTimestamp(void)
{
    return convertTime(lastTimestamp);
}

bhyTimestamp BHYSensor::convertTime(uint32_t ticks)
{
    bhyTimestamp time;

    time.seconds = ticks / 32000;
    time.nanoseconds = (ticks * 31250) - (time.seconds * 1000000000);

    return time;
}

void BHYSensor::parseBuffer(void)
{
    if (nextEventId <= SENSOR_CALLBACK_LIST_NUM && callbacks[nextEventId].invalid != NULL)
    {
        callbacks[nextEventId].typeOnly(nextEventSensorId);
    }

    finishCurrentEvent();
}

void BHYSensor::parseBufferU8(void)
{
#ifdef DEBUG_MODE
    if (eventDump)
        dumpBuffer(nextEventId, nextEventDataSize);
#endif

    uint8_t data = buffer[bufferStart++];

    if (nextEventId <= SENSOR_CALLBACK_LIST_NUM && callbacks[nextEventId].invalid != NULL)
    {
        callbacks[nextEventId].uint8(data, nextEventSensorId);
    }

    finishCurrentEvent();
}

void BHYSensor::parseBufferU16(void)
{
#ifdef DEBUG_MODE
    if (eventDump)
        dumpBuffer(nextEventId, nextEventDataSize);
#endif

    uint16_t data = buffer[bufferStart++];
    data |= buffer[bufferStart++] << 8;

    if (nextEventId <= SENSOR_CALLBACK_LIST_NUM && callbacks[nextEventId].invalid != NULL)
    {
        if (nextEventSensorId == BHY_VS_STEP_COUNTER)
            callbacks[nextEventId].uint16(data, nextEventSensorId);
    }
    else if ((nextEventId == BHY_SID_TIMESTAMP_LSW || nextEventId == BHY_SID_TIMESTAMP_LSW_WAKEUP) &&
             timeCallbacks[1] != NULL)
    {
        lastTimestamp = (lastTimestamp & BHY_MASK_MSW_TIMESTAMP) | data;
        timeCallbacks[1](data);
    }
    else if ((nextEventId == BHY_SID_TIMESTAMP_MSW || nextEventId == BHY_SID_TIMESTAMP_MSW_WAKEUP) &&
             timeCallbacks[2] != NULL)
    {
        lastTimestamp = (lastTimestamp & BHY_MASK_LSW_TIMESTAMP) | (data << 16);
        timeCallbacks[2](data);
    }

    finishCurrentEvent();
}

void BHYSensor::parseBufferScaledFloat(void)
{
#ifdef DEBUG_MODE
    if (eventDump)
        dumpBuffer(nextEventId, nextEventDataSize);
#endif

    uint16_t raw;
    raw = buffer[bufferStart++];
    raw |= buffer[bufferStart++] << 8;
    float scaledVal = 0.0f;

    switch (nextEventSensorId)
    {
    case BHY_VS_TEMPERATURE:
    case BHY_VS_AMBIENT_TEMPERATURE:
        scaledVal = ((float)((int16_t)raw) / 500.0f) + 24.0f;
        break;
    case BHY_VS_LIGHT:
        scaledVal = ((float)raw / 625.0f) + 4096.0f;
        break;
    case BHY_VS_PROXIMITY:
        scaledVal = ((float)raw / 25.0f) + 16384.0f;
        break;
    case BHY_VS_RELATIVE_HUMIDITY:
        scaledVal = (float)raw;
        break;
    }

    if (nextEventId <= SENSOR_CALLBACK_LIST_NUM && callbacks[nextEventId].invalid != NULL)
    {
        callbacks[nextEventId].scaledFloat(scaledVal, nextEventSensorId);
    }

    finishCurrentEvent();
}

void BHYSensor::parseBufferU24(void)
{
#ifdef DEBUG_MODE
    if (eventDump)
        dumpBuffer(nextEventId, nextEventDataSize);
#endif

    uint32_t data;

    data = buffer[bufferStart++];
    data |= buffer[bufferStart++] << 8;
    data |= buffer[bufferStart++] << 16;

    float scaledVal = (float)data / 128.0f;

    if (nextEventId <= SENSOR_CALLBACK_LIST_NUM && callbacks[nextEventId].invalid != NULL)
    {
        callbacks[nextEventId].scaledFloat(scaledVal, nextEventSensorId);
    }

    finishCurrentEvent();
}

void BHYSensor::parseBufferVector(void)
{
    uint16_t temp;
    bhyVector data;

#ifdef DEBUG_MODE
    if (eventDump)
        dumpBuffer(nextEventId, nextEventDataSize);
#endif

    temp = buffer[bufferStart++];
    temp |= buffer[bufferStart++] << 8;
    data.x = static_cast<int16_t>(temp);

    temp = buffer[bufferStart++];
    temp |= buffer[bufferStart++] << 8;
    data.y = static_cast<int16_t>(temp);

    temp = buffer[bufferStart++];
    temp |= buffer[bufferStart++] << 8;
    data.z = static_cast<int16_t>(temp);

    data.status = buffer[bufferStart++];

    switch (nextEventSensorId)
    {
    case BHY_VS_ACCELEROMETER:
        break;
    case BHY_VS_GEOMAGNETIC_FIELD:
        break;
    case BHY_VS_ORIENTATION:
        data.x *= 360.0f;
        data.x /= 32768.0f;
        data.y *= 360.0f;
        data.y /= 32768.0f;
        data.z *= 360.0f;
        data.z /= 32768.0f;
        break;
    case BHY_VS_GYROSCOPE:
        break;
    case BHY_VS_GRAVITY:
        break;
    case BHY_VS_LINEAR_ACCELERATION:
        break;
    }

    if (nextEventId <= SENSOR_CALLBACK_LIST_NUM && callbacks[nextEventId].invalid != NULL)
    {
        callbacks[nextEventId].vector(data, nextEventSensorId);
    }

    finishCurrentEvent();
}

void BHYSensor::parseBuffer(bhyVectorUncalib *data)
{
#ifdef DEBUG_MODE
    if (eventDump)
        dumpBuffer(nextEventId, nextEventDataSize);
#endif

    uint16_t temp;

    temp = buffer[bufferStart++];
    temp |= buffer[bufferStart++] << 8;
    data->x = static_cast<int16_t>(temp);

    temp = buffer[bufferStart++];
    temp |= buffer[bufferStart++] << 8;
    data->y = static_cast<int16_t>(temp);

    temp = buffer[bufferStart++];
    temp |= buffer[bufferStart++] << 8;
    data->z = static_cast<int16_t>(temp);

    temp = buffer[bufferStart++];
    temp |= buffer[bufferStart++] << 8;
    data->x_bias = static_cast<int16_t>(temp);

    temp = buffer[bufferStart++];
    temp |= buffer[bufferStart++] << 8;
    data->y_bias = static_cast<int16_t>(temp);

    temp = buffer[bufferStart++];
    temp |= buffer[bufferStart++] << 8;
    data->z_bias = static_cast<int16_t>(temp);

    data->status = buffer[bufferStart++];

    if (nextEventId <= SENSOR_CALLBACK_LIST_NUM && callbacks[nextEventId].invalid != NULL)
    {
        callbacks[nextEventId].vectorUncalib(data, nextEventSensorId);
    }

    finishCurrentEvent();
}

void BHYSensor::parseBufferQuaternion(void)
{
    uint16_t temp;
    bhyQuaternion data;

#ifdef DEBUG_MODE
    if (eventDump)
        dumpBuffer(nextEventId, nextEventDataSize);
#endif

    temp = buffer[bufferStart++];
    temp |= buffer[bufferStart++] << 8;
    data.x = (int16_t)temp;

    temp = buffer[bufferStart++];
    temp |= buffer[bufferStart++] << 8;
    data.y = (int16_t)temp;

    temp = buffer[bufferStart++];
    temp |= buffer[bufferStart++] << 8;
    data.z = (int16_t)temp;

    temp = buffer[bufferStart++];
    temp |= buffer[bufferStart++] << 8;
    data.w = (int16_t)temp;

    temp = buffer[bufferStart++];
    temp |= buffer[bufferStart++] << 8;
    data.accuracyRadians = (int16_t)temp;

    data.w /= 16384.0f;
    data.x /= 16384.0f;
    data.y /= 16384.0f;
    data.z /= 16384.0f;
    data.accuracyRadians /= 16384.0f;

    if (nextEventId <= SENSOR_CALLBACK_LIST_NUM && callbacks[nextEventId].invalid != NULL)
    {
        callbacks[nextEventId].quaternion(data, nextEventSensorId);
    }

    finishCurrentEvent();
}

void BHYSensor::parseBufferMetaevent(void)
{
#ifdef DEBUG_MODE
    if (eventDump)
        dumpBuffer(nextEventId, nextEventDataSize);
#endif
    bhyMetaEvent data;
    data.id = nextEventId;
    data.type = (bhyMetaEventType)buffer[bufferStart++];
    data.info = buffer[bufferStart++];
    data.data = buffer[bufferStart++];

    uint8_t effectiveType = data.type;
    if (nextEventIsWakeup)
        effectiveType += 16;

    if (effectiveType <= METAEVENT_CALLBACK_LIST_NUM && metaCallbacks[effectiveType] != NULL)
    {
        metaCallbacks[effectiveType](&data, data.type);
    }

    finishCurrentEvent();
}

void BHYSensor::parseBufferDebugEvent()
{
    uint8_t data[25];

    for (int i = 0; i < 25; ++i)
    {
        data[i] = buffer[bufferStart++];
    }

    if (nextEventId <= SENSOR_CALLBACK_LIST_NUM && callbacks[nextEventId].invalid != NULL)
    {
        callbacks[nextEventId].bytes(data, 25, nextEventSensorId);
    }

    finishCurrentEvent();
}

void BHYSensor::parseBuffer(bhyBSX *data)
{
#ifdef DEBUG_MODE
    if (eventDump)
        dumpBuffer(nextEventId, nextEventDataSize);
#endif

    uint32_t temp32;

    temp32 = buffer[bufferStart++];
    temp32 |= buffer[bufferStart++] << 8;
    temp32 |= buffer[bufferStart++] << 16;
    temp32 |= buffer[bufferStart++] << 24;
    data->x = static_cast<int32_t>(temp32);

    temp32 = buffer[bufferStart++];
    temp32 |= buffer[bufferStart++] << 8;
    temp32 |= buffer[bufferStart++] << 16;
    temp32 |= buffer[bufferStart++] << 24;
    data->y = static_cast<int32_t>(temp32);

    temp32 = buffer[bufferStart++];
    temp32 |= buffer[bufferStart++] << 8;
    temp32 |= buffer[bufferStart++] << 16;
    temp32 |= buffer[bufferStart++] << 24;
    data->z = static_cast<int32_t>(temp32);

    temp32 = buffer[bufferStart++];
    temp32 |= buffer[bufferStart++] << 8;
    temp32 |= buffer[bufferStart++] << 16;
    temp32 |= buffer[bufferStart++] << 24;
    data->timestamp = temp32;

    finishCurrentEvent();
}

void BHYSensor::parseBufferActivity(void)
{
#ifdef DEBUG_MODE
    if (eventDump)
        dumpBuffer(nextEventId, nextEventDataSize);
#endif

    bhyActivityEvent actEvent;

    uint16_t data = buffer[bufferStart++];
    data |= buffer[bufferStart++] << 8;

    actEvent.value = data;

    if (nextEventId <= SENSOR_CALLBACK_LIST_NUM && callbacks[nextEventId].invalid != NULL)
    {
        callbacks[nextEventId].activity(actEvent, nextEventSensorId);
    }

    finishCurrentEvent();
}

int8_t BHYSensor::configVirtualSensor(bhyVirtualSensor sensorId,
                                      bool wakeup,
                                      bhyFlush flushSensor,
                                      uint16_t samplingFreq,
                                      uint16_t maxReportLatency,
                                      uint16_t changeSensitivity,
                                      uint16_t dynamicRange)
{
    /* checks if sensor id is in range */
    if ((uint8_t)sensorId >= BHY_MAX_SENSOR_ID)
        return status = BHY_E_OUT_OF_RANGE;

    /* computes the sensor id */
    uint8_t effectiveId = sensorId;
    if (wakeup)
        effectiveId += BHY_SID_WAKEUP_OFFSET;

    /* flush the fifo if requested */
    switch (flushSensor)
    {
    case BHY_FLUSH_ONE:
        status = setFifoFlush(effectiveId);
        break;
    case BHY_FLUSH_ALL:
        status = setFifoFlush(VS_FLUSH_ALL);
        break;
    case BHY_FLUSH_NONE:
        break;
    default:

        return BHY_E_OUT_OF_RANGE;
    }

    if (status < 0)
        return status;

    /* computes the param page as sensor_id + 0xC0 (sensor parameter write)*/
    effectiveId += BHY_SENSOR_PARAMETER_WRITE;

    commBuffer[0] = (samplingFreq) & 0xFF;
    commBuffer[1] = (samplingFreq >> 8) & 0xFF;
    commBuffer[2] = (maxReportLatency) & 0xFF;
    commBuffer[3] = (maxReportLatency >> 8) & 0xFF;
    commBuffer[4] = (changeSensitivity) & 0xFF;
    commBuffer[5] = (changeSensitivity >> 8) & 0xFF;
    commBuffer[6] = (dynamicRange) & 0xFF;
    commBuffer[7] = (dynamicRange >> 8) & 0xFF;

    return writeConfiguration(effectiveId);
}

int8_t BHYSensor::configVirtualSensor(bhyVirtualSensor sensorId,
                                      bool wakeup,
                                      bhyFlush flushSensor,
                                      bhySensorConfiguration configuration)
{
    return configVirtualSensor(sensorId,
                               wakeup,
                               flushSensor,
                               configuration.sampleRate,
                               configuration.maxReportLatency,
                               configuration.changeSensitivity,
                               configuration.dynamicRange);
}

int8_t BHYSensor::disableVirtualSensor(bhyVirtualSensor sensorId, bool wakeup)
{
    // Checks if sensor ID is in range
    if (sensorId >= BHY_MAX_SENSOR_ID)
        return status = BHY_E_OUT_OF_RANGE;

    /* computes the param page as */
    /* wakeup_status + sensor_id + 0xC0 (sensor parameter write) */
    uint8_t effectiveId = sensorId;
    if (wakeup)
        effectiveId += BHY_SID_WAKEUP_OFFSET;
    effectiveId += BHY_SENSOR_PARAMETER_WRITE;

    for (uint8_t i = 0; i < BHY_SENSOR_CONFIGURATION_SIZE; ++i)
        commBuffer[i] = 0;

    return writeConfiguration(effectiveId);
}

int8_t BHYSensor::installSensorCallback(bhyVirtualSensor id, bool wakeup, bhyCallbackFptrTypeOnly callback)
{
    if (id >= BHY_MAX_SENSOR_ID)
        return status = BHY_E_OUT_OF_RANGE;

    uint8_t effectiveId = id;
    if (wakeup)
        effectiveId += BHY_SID_WAKEUP_OFFSET;

    if (BHY_SENSOR_EVENT_TYPE != getDataType(effectiveId))
        return status = BHY_E_INVALID_INPUT;

    callbacks[effectiveId].typeOnly = callback;

    return status = BHY_OK;
}

int8_t BHYSensor::installSensorCallback(bhyVirtualSensor id, bool wakeup, bhyCallbackFptrU8 callback)
{
    if (id >= BHY_MAX_SENSOR_ID)
        return status = BHY_E_OUT_OF_RANGE;

    uint8_t effectiveId = id;
    if (wakeup)
        effectiveId += BHY_SID_WAKEUP_OFFSET;

    if (BHY_SCALAR_TYPE_U8 != getDataType(effectiveId))
        return status = BHY_E_INVALID_INPUT;

    callbacks[effectiveId].uint8 = callback;

    return status = BHY_OK;
}

int8_t BHYSensor::installSensorCallback(bhyVirtualSensor id, bool wakeup, bhyCallbackFptrU16 callback)
{
    if (id >= BHY_MAX_SENSOR_ID)
        return status = BHY_E_OUT_OF_RANGE;

    uint8_t effectiveId = id;
    if (wakeup)
        effectiveId += BHY_SID_WAKEUP_OFFSET;

    if (BHY_SCALAR_TYPE_U16 != getDataType(effectiveId))
        return status = BHY_E_INVALID_INPUT;

    callbacks[effectiveId].uint16 = callback;

    return status = BHY_OK;
}

int8_t BHYSensor::installSensorCallback(bhyVirtualSensor id, bool wakeup, bhyCallbackFptrActivity callback)
{
    if (id >= BHY_MAX_SENSOR_ID)
        return status = BHY_E_OUT_OF_RANGE;

    uint8_t effectiveId = id;
    if (wakeup)
        effectiveId += BHY_SID_WAKEUP_OFFSET;

    if (id != BHY_VS_ACTIVITY_RECOGNITION)
        return status = BHY_E_INVALID_INPUT;

    callbacks[effectiveId].activity = callback;

    return status = BHY_OK;
}

int8_t BHYSensor::installSensorCallback(bhyVirtualSensor id, bool wakeup, bhyCallbackFptrFloat callback)
{
    if (id >= BHY_MAX_SENSOR_ID)
        return status = BHY_E_OUT_OF_RANGE;

    uint8_t effectiveId = id;
    if (wakeup)
        effectiveId += BHY_SID_WAKEUP_OFFSET;

    switch (id)
    {
    case BHY_VS_TEMPERATURE:
    case BHY_VS_AMBIENT_TEMPERATURE:
    case BHY_VS_LIGHT:
    case BHY_VS_PROXIMITY:
    case BHY_VS_RELATIVE_HUMIDITY:
    case BHY_VS_PRESSURE:
        callbacks[effectiveId].scaledFloat = callback;
        break;
    default:

        return status = BHY_E_INVALID_INPUT;
    }

    return status = BHY_OK;
}

int8_t BHYSensor::installSensorCallback(bhyVirtualSensor id, bool wakeup, bhyCallbackFptrBytes callback)
{
    if (id >= BHY_MAX_SENSOR_ID)
        return status = BHY_E_OUT_OF_RANGE;

    uint8_t effectiveId = id;
    if (wakeup)
        effectiveId += BHY_SID_WAKEUP_OFFSET;

    if (BHY_DEBUG_TYPE != getDataType(effectiveId))
        return status = BHY_E_INVALID_INPUT;

    callbacks[effectiveId].bytes = callback;

    return status = BHY_OK;
}

int8_t BHYSensor::installSensorCallback(bhyVirtualSensor id, bool wakeup, bhyCallbackFptrQuaternion callback)
{
    if (id >= BHY_MAX_SENSOR_ID)
        return status = BHY_E_OUT_OF_RANGE;

    uint8_t effectiveId = id;
    if (wakeup)
        effectiveId += BHY_SID_WAKEUP_OFFSET;

    if (BHY_QUATERNION_TYPE != getDataType(effectiveId))
        return status = BHY_E_INVALID_INPUT;

    callbacks[effectiveId].quaternion = callback;

    return status = BHY_OK;
}

int8_t BHYSensor::installSensorCallback(bhyVirtualSensor id, bool wakeup, bhyCallbackFptrVector callback)
{
    if (id >= BHY_MAX_SENSOR_ID)
        return status = BHY_E_OUT_OF_RANGE;

    uint8_t effectiveId = id;
    if (wakeup)
        effectiveId += BHY_SID_WAKEUP_OFFSET;

    if (BHY_VECTOR_TYPE != getDataType(effectiveId))
        return status = BHY_E_INVALID_INPUT;

    callbacks[effectiveId].vector = callback;

    return status = BHY_OK;
}

int8_t BHYSensor::installSensorCallback(bhyVirtualSensor id, bool wakeup, bhyCallbackFptrVectorUncalib callback)
{
    if (id >= BHY_MAX_SENSOR_ID)
        return status = BHY_E_OUT_OF_RANGE;

    uint8_t effectiveId = id;
    if (wakeup)
        effectiveId += BHY_SID_WAKEUP_OFFSET;

    if (BHY_VECTOR_TYPE_UNCALIBRATED != getDataType(effectiveId))
        return status = BHY_E_INVALID_INPUT;

    callbacks[effectiveId].vectorUncalib = callback;

    return status = BHY_OK;
}

int8_t BHYSensor::uninstallSensorCallback(bhyVirtualSensor id, bool wakeup)
{

    if (id >= BHY_MAX_SENSOR_ID)
        return status = BHY_E_OUT_OF_RANGE;

    uint8_t effectiveId = id;
    if (wakeup)
        effectiveId += BHY_SID_WAKEUP_OFFSET;

    callbacks[effectiveId].invalid = NULL;

    return status = BHY_OK;
}

int8_t BHYSensor::installMetaCallback(bhyMetaEventType type, bool wakeup, bhyCallbackFptrMetaEvent callback)
{
    // Checks if sensor ID is in range
    if (type >= BHY_MAX_SENSOR_ID)
        return status = BHY_E_OUT_OF_RANGE;

    /* computes the param page as */
    uint8_t effectiveType = type;
    if (wakeup)
        effectiveType += 16;

    //if (callbacks[effectiveType] != NULL) return status = BHY_E_OUT_OF_RANGE; // There is already a callback installed

    metaCallbacks[effectiveType] = callback;

    return status = BHY_OK;
}

int8_t BHYSensor::uninstallMetaCallback(bhyMetaEventType type, bool wakeup)
{
    // Checks if sensor ID is in range
    if (type >= BHY_MAX_SENSOR_ID)
        return status = BHY_E_OUT_OF_RANGE;

    /* computes the param page as */
    uint8_t effectiveType = type;
    if (wakeup)
        effectiveType += 16;

    metaCallbacks[effectiveType] = NULL;

    return status = BHY_OK;
}

int8_t BHYSensor::installTimeCallback(bool lsw, bhyCallbackFptrTimestamp callback)
{
    uint8_t effectiveType = (lsw) ? 1 : 2;

    timeCallbacks[effectiveType] = callback;

    return status = BHY_OK;
}

int8_t BHYSensor::uninstallTimeCallback(bool lsw)
{
    /* computes the param page as */
    uint8_t effectiveType = (lsw) ? 1 : 2;

    timeCallbacks[effectiveType] = NULL;

    return status = BHY_OK;
}

int8_t BHYSensor::writeParameterPage(bhyPage page, uint8_t parameter)
{
    status = write(BHY_REG_PARAMETER_WRITE_BUFFER_ZERO, commBuffer, BHY_SENSOR_CONFIGURATION_SIZE);
    if (status < 0)
        return status; //

    return pageSelect(page, parameter);
}

int8_t BHYSensor::readParameterPage(bhyPage page, uint8_t parameter)
{
    if (pageSelect(page, parameter) < 0)
        return status;

    uint8_t size = BHY_SENSOR_CONFIGURATION_SIZE; //

    if (page == BHY_SYSTEM_PAGE &&
        (parameter == BHY_SENSOR_STATUS_BANK_0 || parameter == BHY_SENSOR_STATUS_BANK_1 ||
         parameter == BHY_SENSOR_STATUS_BANK_2 || parameter == BHY_SENSOR_STATUS_BANK_3))
    {
        size = BHY_SENSOR_INFORMATION_SIZE;
    }
    else if (page == BHY_SYSTEM_PAGE && (parameter == BHY_PHYSICAL_SENSOR_STATUS))
    {
        size = BHY_PHYSICAL_SENSOR_INFORMATION_SIZE;
    }
    else if (page == BHY_SENSORS_PAGE)
    {
        size = BHY_SENSOR_INFORMATION_SIZE;
    }

    status = read(BHY_REG_PARAMETER_READ_BUFFER_ZERO, commBuffer, size);

    return status;
}

int8_t BHYSensor::writeConfiguration(uint8_t parameter)
{
    return writeParameterPage(BHY_SENSORS_PAGE, parameter);
}

int8_t BHYSensor::getConfiguration(bhyVirtualSensor sensor, bool wakeup, bhySensorConfiguration *configuration)
{
    if (sensor == BHY_VS_NOT_USED || sensor == BHY_VS_INVALID)
        return status;
    uint8_t parameter = BHY_SENSOR_PARAMETER_WRITE + (wakeup) ? sensor + BHY_SID_WAKEUP_OFFSET : sensor; // 64 in documentation, but elsewhere SENSOR_PARAMETER_WRITE

    if (pageSelect(BHY_SENSORS_PAGE, parameter) < 0)
        return status;

    status = write(BHY_REG_PARAMETER_REQUEST_ADDR, parameter);

    status = read(BHY_REG_PARAMETER_READ_BUFFER_ZERO, commBuffer, BHY_SENSOR_CONFIGURATION_SIZE); //BHY_READ_BHY_FIFO_BUFFER_SIZE

    /* sample rate information */
    configuration->sampleRate = (((uint16_t)commBuffer[1] << 8) | commBuffer[0]);

    /* max report latency information */
    configuration->maxReportLatency = (((uint16_t)commBuffer[3] << 8) | commBuffer[2]);

    /* sensitivity information */
    configuration->changeSensitivity = (((uint16_t)commBuffer[5] << 8) | commBuffer[4]);

    /* dynamic range information */
    configuration->dynamicRange = (((uint16_t)commBuffer[7] << 8) | commBuffer[6]);

    return status;
}

int8_t BHYSensor::pageSelect(bhyPage page, uint8_t parameter)
{
    status = write(BHY_REG_PARAMETER_PAGE_SELECT_ADDR, page);
    if (status < 0)
        return status;

    status = write(BHY_REG_PARAMETER_REQUEST_ADDR, parameter);
    if (status < 0)
        return status;

    uint8_t ack = 0;
    for (uint8_t i = 0; i < 250; ++i)
    {
        status = read(BHY_REG_PARAMETER_ACKNOWLEDGE_ADDR, &ack);
        if (status < 0)
            return status;

        if (ack == parameter)
            return status = BHY_OK;
        else if (ack == 0x80)
        {
            delay(50);
            status = BHY_ERROR;
        }
        else
            delay(1); // device is not ready yet
    }
    if (status < 0)
        return status;
}

int8_t BHYSensor::getPhysicalSensorStatus(bhyPhysicalStatus *accel, bhyPhysicalStatus *gyro, bhyPhysicalStatus *mag)
{
    status = readParameterPage(BHY_SYSTEM_PAGE, BHY_PHYSICAL_SENSOR_STATUS);

    if (accel)
    {
        accel->sampleRate = (((uint16_t)commBuffer[1] << 8) | commBuffer[0]);
        accel->dynamicRange = (((uint16_t)commBuffer[3] << 8) | commBuffer[2]);
        accel->flag = commBuffer[4];
    }

    if (gyro)
    {
        gyro->sampleRate = (((uint16_t)commBuffer[6] << 8) | commBuffer[5]);
        gyro->dynamicRange = (((uint16_t)commBuffer[8] << 8) | commBuffer[7]);
        gyro->flag = commBuffer[9];
    }

    if (mag)
    {
        mag->sampleRate = (((uint16_t)commBuffer[11] << 8) | commBuffer[10]);
        mag->dynamicRange = (((uint16_t)commBuffer[13] << 8) | commBuffer[12]);
        mag->flag = commBuffer[14];
    }

    return status = BHY_OK;
}

int8_t BHYSensor::getSensorInformation(bhyVirtualSensor sensor, bool wakeup, bhySensorInformation *information)
{
    if (sensor == BHY_VS_NOT_USED || sensor == BHY_VS_INVALID)
        return status;
    uint8_t effectiveType = (wakeup) ? sensor + BHY_SID_WAKEUP_OFFSET : sensor;

    // input as page 3 and parameter request for sensor page
    status = readParameterPage(BHY_SENSORS_PAGE, effectiveType);

    information->type = commBuffer[0];
    information->driver_id = commBuffer[1];
    information->driver_version = commBuffer[2];
    information->power = commBuffer[3];
    information->max_range = (((uint16_t)commBuffer[5] << 8) | commBuffer[4]);
    information->resolution = (((uint16_t)commBuffer[7] << 8) | commBuffer[6]);
    information->max_rate = (((uint16_t)commBuffer[9] << 8) | commBuffer[8]);
    information->fifo_reserved = (((uint16_t)commBuffer[11] << 8) | commBuffer[10]);
    information->fifo_max = (((uint16_t)commBuffer[13] << 8) | commBuffer[12]);
    information->event_size = commBuffer[14];
    information->min_rate = commBuffer[15];

    return status;
}

int8_t BHYSensor::getSensorStatus(bhyVirtualSensor sensor, bool wakeup, bhySensorStatus *sensorStatus)
{
    if (sensor == BHY_VS_NOT_USED || sensor == BHY_VS_INVALID)
        return status;
    uint8_t effectiveType = (wakeup) ? sensor + BHY_SID_WAKEUP_OFFSET : sensor;
    bhySystemPageParameter parameter =
        (bhySystemPageParameter)(BHY_SENSOR_STATUS_BANK_0 + effectiveType / (BHY_SID_WAKEUP_OFFSET / 2));
    uint8_t byte = effectiveType % (BHY_SID_WAKEUP_OFFSET / 2);

    status = readParameterPage(BHY_SYSTEM_PAGE, parameter);

    sensorStatus->value = commBuffer[byte];

    return status;
}

int8_t BHYSensor::getParameterPageSelect(uint8_t page_select, uint8_t *parameter_page)
{
    uint8_t data = 0;

    page_select &= 1;

    /* read the parameter page information*/
    status = read(BHY_REG_PARAMETER_PAGE_SELECT_ADDR, &data);

    switch (page_select)
    {
    case BHY_PAGE_SELECT_PARAMETER_PAGE:     //0
        *parameter_page = data & 0x0F;
        break;
    case BHY_PAGE_SELECT_PARAMETER_SIZE:     //1
        *parameter_page = (data >> 4) & 0x0F;
        break;
    }

    return status;
}

int8_t BHYSensor::setParameterPageSelect(uint8_t value)
{
    return status = write(BHY_REG_PARAMETER_PAGE_SELECT_ADDR, value);
}

int8_t BHYSensor::getHostInterfaceControl(bhyHostInterfaceControl *interfaceControl)
{
    // read the host interrupt status
    status = read(BHY_REG_HOST_INTERFACE_CONTROL_ADDR, &interfaceControl->value);

    return status;
}

int8_t BHYSensor::getValue(uint8_t *value)
{
    return status = read(BHY_REG_PARAMETER_REQUEST_ADDR, value);
}

int8_t BHYSensor::setValue(uint8_t value)
{
    return status = write(BHY_REG_PARAMETER_REQUEST_ADDR, value);
}

uint16_t BHYSensor::getFifoSize(bool wakeup)
{
    status = readParameterPage(BHY_SYSTEM_PAGE, BHY_FIFO_CONTROL);

    if (status != BHY_OK)
        return 0;

    if (wakeup)
        return (((uint16_t)commBuffer[3] << 8) | commBuffer[2]);
    else
        return (((uint16_t)commBuffer[7] << 8) | commBuffer[6]);
}

uint16_t BHYSensor::getFifoWatermark(bool wakeup)
{
    status = readParameterPage(BHY_SYSTEM_PAGE, BHY_FIFO_CONTROL);

    if (status != BHY_OK)
        return status;

    return (wakeup) ? ((uint16_t)commBuffer[1] << 8) | commBuffer[0] : ((uint16_t)commBuffer[5] << 8) | commBuffer[4];
}

int8_t BHYSensor::setFifoWatermark(bool wakeup, uint16_t bytes)
{
    status = readParameterPage(BHY_SYSTEM_PAGE, BHY_FIFO_CONTROL);

    if (status != BHY_OK)
        return status;

    if (wakeup)
    {
        commBuffer[0] = (bytes & 0xFF);
        commBuffer[1] = ((bytes >> 8) & 0xFF);
    }
    else
    {
        commBuffer[4] = (bytes & 0xFF);
        commBuffer[5] = ((bytes >> 8) & 0xFF);
    }
    status = writeParameterPage(BHY_SYSTEM_PAGE, 0x80 + 2);

    return status;
}

int8_t BHYSensor::getSensorStatusBank(uint8_t parameter, uint8_t sensor_type)
{
    uint8_t data = 0;

    status = readParameterPage(BHY_SYSTEM_PAGE, parameter);

    if (status != BHY_OK)
        return status;

    sensorStatusBank.value = commBuffer[sensor_type];

    return status;
}

int8_t BHYSensor::readTimestamps(uint32_t *hostIrq, uint32_t *current)
{
    status = readParameterPage(BHY_SYSTEM_PAGE, BHY_HOST_IRQ_TIMESTAMP);

    if (status != BHY_OK)
        return status;

    *current = (((((((uint32_t)commBuffer[3] << 8) | commBuffer[2]) << 8) | commBuffer[1]) << 8) | commBuffer[0]);
    *hostIrq = (((((((uint32_t)commBuffer[7] << 8) | commBuffer[6]) << 8) | commBuffer[5]) << 8) | commBuffer[4]);

    return status;
}

int8_t BHYSensor::getSoftPassThrough(bhySoftPassThrough *data, uint8_t parameter)
{
    status = readParameterPage(BHY_SOFT_PASS_THROUGH_PAGE, parameter);

    data->address = commBuffer[0];
    data->start = commBuffer[1];
    data->length = commBuffer[2];
    data->status = commBuffer[3];
    data->data[0] = commBuffer[4];
    data->data[1] = commBuffer[5];
    data->data[2] = commBuffer[6];
    data->data[3] = commBuffer[7];

    return status;
}

int8_t BHYSensor::setSoftPassThrough(bhySoftPassThrough *data, uint8_t parameter)
{
    commBuffer[0] = data->address;
    commBuffer[1] = data->start;
    commBuffer[2] = data->length;
    commBuffer[3] = data->status;
    commBuffer[4] = data->data[0];
    commBuffer[5] = data->data[1];
    commBuffer[6] = data->data[2];
    commBuffer[7] = data->data[3];

    status = writeParameterPage(BHY_SOFT_PASS_THROUGH_PAGE, parameter);

    return status;
}

int8_t BHYSensor::read(uint8_t regAddr, uint8_t *data, uint16_t length)
{
    if (length > ARDUINO_I2C_BUFFER_SIZE)
    {
#ifdef DEBUG_MODE
        if (debugOut && (debugLevel >= BHY_ERROR))
        {
            debugOut->print("Unable to read ");
            debugOut->print(length);
            debugOut->print("bytes from device 0x");
            debugOut->print(deviceId, HEX);
            debugOut->print(" register 0x");
            if (regAddr < 16)
                debugOut->print("0");
            debugOut->print(regAddr, HEX);
            debugOut->print(" [max read length is ");
            debugOut->print(ARDUINO_I2C_BUFFER_SIZE);
            debugOut->println("]");
        }
#endif

        return -1;
    }

#ifdef DEBUG_MODE
    if (debugOut && (debugLevel >= BHY_DEBUG) && commDump)
    {
        debugOut->print("Reading from\t0x");
        debugOut->print(deviceId, HEX);
        debugOut->print("\tregister 0x");
        if (regAddr < 16)
            debugOut->print("0");
        debugOut->println(regAddr, HEX);
    }
#endif

    // Begin I2C communication with provided I2C address
    Wire.beginTransmission(deviceId);

    // The following code switches out Wire library code for the older
    // implementation if using the older version of the library.
    // In either case, the register address to be read is sent to
    // the sensor.
#if ARDUINO >= 100
    Wire.write(regAddr); // This is for the modern Wire library
#else
    Wire.send(regAddr);         // This is for the older Wire library
#endif

    // Done writting, end the transmission
    int8_t returned = Wire.endTransmission();

    /*
       0:success
       1:data too long to fit in transmit buffer
       2:received NACK on transmit of address
       3:received NACK on transmit of data
       4:other error
     */

    if (returned)
    {
#ifdef DEBUG_MODE
        if (debugOut && (debugLevel >= BHY_ERROR))
        {
            debugOut->print("Communication error: ");
            switch (returned)
            {
            case 1:
                debugOut->println("Data too long to fit in transmit buffer.");
                break;
            case 2:
                debugOut->println("received NACK on transmit of address.");
                break;
            case 3:
                debugOut->println("received NACK on transmit of data.");
                break;
            case 4:
                debugOut->println("Unspecified error.");
                break;
            default:
                debugOut->print("Unexpected Wire.endTransmission() return code: ");
                debugOut->println(returned);
            }
        }
#endif

        return returned;
    }

    // Requests the required number of bytes from the sensor
    Wire.requestFrom((int)deviceId, (int)length);
    //uint16_t toRead = length;
    //Wire.requestFrom((int) deviceId, (int) ((toRead > ARDUINO_I2C_BUFFER_SIZE) ? ARDUINO_I2C_BUFFER_SIZE: toRead));

    uint16_t i;
    // Reads the requested number of bytes into the provided array
    for (i = 0; (i < length) && Wire.available(); i++)
    {
#if ARDUINO >= 100
        data[i] = Wire.read(); // This is for the modern Wire library
#else
        data[i] = Wire.receive();         // This is for the older Wire library
#endif

#ifdef DEBUG_MODE
        if (debugOut && (debugLevel >= BHY_DEBUG) && commDump)
        {
            debugOut->print("\t0x");
            if (data[i] < 16)
                debugOut->print("0");
            debugOut->print(data[i], HEX);
            if (i == length - 1)
                debugOut->println("");
            else if (i % 8 == 7)
                debugOut->println(",");
            else
                debugOut->print(",");
        }
#endif
    }

    if (i < length)
    {
#ifdef DEBUG_MODE
        if (debugOut && (debugLevel >= BHY_ERROR))
        {
            debugOut->print("Communication error: Failed to read ");
            debugOut->print(length - i);
            debugOut->println(" bytes.");
        }
#endif

        return -2;
    }

    // This must return 0 on success, any other value will be interpreted as a communication failure.
    return 0;
}

int8_t BHYSensor::write(uint8_t regAddr, uint8_t *data, uint16_t length)
{
    if (length >= ARDUINO_I2C_BUFFER_SIZE)
    {
#ifdef DEBUG_MODE
        if (debugOut && (debugLevel >= BHY_ERROR))
        {
            debugOut->print("Unable to write ");
            debugOut->print(length);
            debugOut->print("bytes to device 0x");
            debugOut->print(deviceId, HEX);
            debugOut->print(" register 0x");
            if (regAddr < 16)
                debugOut->print("0");
            debugOut->print(regAddr, HEX);
            debugOut->print(" [max write length is ");
            debugOut->print(ARDUINO_I2C_BUFFER_SIZE - 1);
            debugOut->println("]");
        }
#endif

        return -1;
    }

#ifdef DEBUG_MODE
    if (debugOut && (debugLevel >= BHY_DEBUG) && commDump)
    {
        debugOut->print("Writing to\t0x");
        debugOut->print(deviceId, HEX);
        debugOut->print("\tregister 0x");
        if (regAddr < 16)
            debugOut->print("0");
        debugOut->println(regAddr, HEX);
    }
#endif //DEBUG_MODE

    // Begin I2C communication with provided I2C address
    Wire.beginTransmission(deviceId);

    // The following code switches out Wire library code for the older
    // implementation if using the older version of the library.
    // In either case, the register address to be written to is sent to
    // the sensor.
#if ARDUINO >= 100
    Wire.write(regAddr); // This is for the modern Wire library
#else
    Wire.send(regAddr);         // This is for the older Wire library
#endif

    uint16_t i;
    // Writes the requested number of bytes from the provided array
    for (i = 0; i < length; i++)
    {

#ifdef DEBUG_MODE
        if (debugOut && (debugLevel >= BHY_DEBUG) && commDump)
        {
            debugOut->print("\t0x");
            if (data[i] < 16)
                debugOut->print("0");
            debugOut->print(data[i], HEX);
            if (i == length - 1)
                debugOut->println("");
            else if (i % 8 == 7)
                debugOut->println(",");
            else
                debugOut->print(",");
        }
#endif

#if ARDUINO >= 100
        Wire.write(data[i]); // This is for the modern Wire library
#else
        Wire.send(data[i]);         // This is for the older Wire library
#endif
    }

    // Done writting, end the transmission
    int8_t returned = Wire.endTransmission();

#ifdef DEBUG_MODE
    if (returned && debugOut && (debugLevel >= BHY_ERROR))
    {
        debugOut->print("Communication error: ");
        switch (returned)
        {
        case 1:
            debugOut->println("Data too long to fit in transmit buffer.");
            break;
        case 2:
            debugOut->println("received NACK on transmit of address.");
            break;
        case 3:
            debugOut->println("received NACK on transmit of data.");
            break;
        case 4:
            debugOut->println("Unspecified error.");
            break;
        default:
            debugOut->print("Unexpected Wire.endTransmission() return code: ");
            debugOut->println(returned);
        }
    }
#endif

    // This must return 0 on sucess, any other value will be interpretted as a communication failure.
    return returned;
}

int8_t BHYSensor::read(uint8_t regAddr, uint8_t *data)
{
    return read(regAddr, data, 1);
}

int8_t BHYSensor::write(uint8_t regAddr, uint8_t data)
{
    return write(regAddr, &data, 1);
}

uint8_t BHYSensor::readByte(uint8_t regAddr)
{
    uint8_t data;

    int8_t returned = read(regAddr, &data, 1);

    return (returned) ? data : 0;
}

uint16_t BHYSensor::readShort(uint8_t regAddr)
{
    uint8_t data[2] = { 0, };

    int8_t returned = read(regAddr, data, 2);

    if (returned)
        return 0;

    uint16_t result = ((uint16_t)data[1] << 8) | data[0];

#ifdef DEBUG_MODE
    if (debugOut && (debugLevel >= BHY_INFORMATIVE) && commDump)
    {
        debugOut->print("Result:\t0x");
        if (result < 0x1000)
            debugOut->print("0");
        if (result < 0x100)
            debugOut->print("0");
        if (result < 0x10)
            debugOut->print("0");
        debugOut->print(result, HEX);
        debugOut->println("");
    }
#endif

    return result;
}

uint32_t BHYSensor::readInteger(uint8_t regAddr)
{
    uint8_t data[4] = { 0, };

    int8_t returned = read(regAddr, data, 4);

    if (returned)
        return 0;

    uint32_t result =
        (uint32_t)(((uint32_t)data[3] << 24) | ((uint32_t)data[2] << 16) | ((uint32_t)data[1] << 8) |
                   ((uint32_t)data[0]));

#ifdef DEBUG_MODE
    if (debugOut && (debugLevel >= BHY_INFORMATIVE) && commDump)
    {
        debugOut->print("Result:\t0x");
        if (result < 0x10000000)
            debugOut->print("0");
        if (result < 0x1000000)
            debugOut->print("0");
        if (result < 0x100000)
            debugOut->print("0");
        if (result < 0x10000)
            debugOut->print("0");
        if (result < 0x1000)
            debugOut->print("0");
        if (result < 0x100)
            debugOut->print("0");
        if (result < 0x10)
            debugOut->print("0");
        debugOut->print(result, HEX);
        debugOut->println("");
    }
#endif

    return result;
}

int8_t BHYSensor::writeByte(uint8_t regAddr, uint8_t data)
{
    return write(regAddr, &data, 1);
}

int8_t BHYSensor::writeShort(uint8_t regAddr, uint16_t data)
{
    uint8_t bytes[2];

    bytes[0] = (uint8_t)(data & 0xFF);
    bytes[1] = (uint8_t)((data >> 8) & 0xFF);

    return write(regAddr, bytes, 2);
}

int8_t BHYSensor::writeInteger(uint8_t regAddr, uint32_t data)
{
    uint8_t bytes[4];

    bytes[0] = (uint8_t)(data & 0xFF);
    data = data >> 8;
    bytes[1] = (uint8_t)(data & 0xFF);
    data = data >> 8;
    bytes[2] = (uint8_t)(data & 0xFF);
    data = data >> 8;
    bytes[3] = (uint8_t)(data & 0xFF);

    return write(regAddr, bytes, 2);
}

String BHYSensor::getSensorName(uint8_t type)
{
    // Order is by non-wakeup ID order
    switch (type)
    {
    case BHY_SID_PADDING:

        return "Padding";
    case BHY_SID_ACCELEROMETER:

        return "Accelerometer (Non Wake-up)";
    case BHY_SID_ACCELEROMETER_WAKEUP:

        return "Accelerometer (Wake-up)";
    case BHY_SID_MAGNETOMETER:

        return "Magnetometer (Non Wake-up)";
    case BHY_SID_MAGNETOMETER_WAKEUP:

        return "Magnetometer (Wake-up)";
    case BHY_SID_ORIENTATION:

        return "Orientation (Non Wake-up)";
    case BHY_SID_ORIENTATION_WAKEUP:

        return "Orientation (Wake-up)";
    case BHY_SID_GYROSCOPE:

        return "Gyroscope (Non Wake-up)";
    case BHY_SID_GYROSCOPE_WAKEUP:

        return "Gyroscope (Wake-up)";
    case BHY_SID_LIGHT:

        return "Light (Non Wake-up)";
    case BHY_SID_LIGHT_WAKEUP:

        return "Light (Wake-up)";
    case BHY_SID_BAROMETER:

        return "Barometer (Non Wake-up)";
    case BHY_SID_BAROMETER_WAKEUP:

        return "Barometer (Wake-up)";
    case BHY_SID_TEMPERATURE:

        return "Temperature (Non Wake-up)";
    case BHY_SID_TEMPERATURE_WAKEUP:

        return "Temperature (Wake-up)";
    case BHY_SID_PROXIMITY:

        return "Proximity (Non Wake-up)";
    case BHY_SID_PROXIMITY_WAKEUP:

        return "Proximity (Wake-up)";
    case BHY_SID_GRAVITY:

        return "Gravity (Non Wake-up)";
    case BHY_SID_GRAVITY_WAKEUP:

        return "Gravity (Wake-up)";
    case BHY_SID_LINEAR_ACCELERATION:

        return "Linear Acceleration (Non Wake-up)";
    case BHY_SID_LINEAR_ACCELERATION_WAKEUP:

        return "Linear Acceleration (Wake-up)";
    case BHY_SID_ROTATION_VECTOR:

        return "Rotation Vector (Non Wake-up)";
    case BHY_SID_ROTATION_VECTOR_WAKEUP:

        return "Rotation Vector (Wake-up)";
    case BHY_SID_HUMIDITY:

        return "Humidity (Non Wake-up)";
    case BHY_SID_HUMIDITY_WAKEUP:

        return "Humidity (Wake-up)";
    case BHY_SID_AMBIENT_TEMPERATURE:

        return "Ambient Temperature (Non Wake-up)";
    case BHY_SID_AMBIENT_TEMPERATURE_WAKEUP:

        return "Ambient Temperature (Wake-up)";
    case BHY_SID_UNCALIBRATED_MAGNETOMETER:

        return "Magnetometer (Non Wake-up)";
    case BHY_SID_UNCALIBRATED_MAGNETOMETER_WAKEUP:

        return "Magnetometer (Wake-up)";
    case BHY_SID_GAME_ROTATION_VECTOR:

        return "Game Rotation (Non Wake-up)";
    case BHY_SID_GAME_ROTATION_VECTOR_WAKEUP:

        return "Game Rotation (Wake-up)";
    case BHY_SID_UNCALIBRATED_GYROSCOPE:

        return "Gyroscope (Non Wake-up)";
    case BHY_SID_UNCALIBRATED_GYROSCOPE_WAKEUP:

        return "Gyroscope (Wake-up)";
    case BHY_SID_SIGNIFICANT_MOTION:

        return "Significant Motion (Non Wake-up)";
    case BHY_SID_SIGNIFICANT_MOTION_WAKEUP:

        return "Significant Motion (Wake-up)";
    case BHY_SID_STEP_DETECTOR:

        return "Step Detector (Non Wake-up)";
    case BHY_SID_STEP_DETECTOR_WAKEUP:

        return "Step Detector (Wake-up)";
    case BHY_SID_STEP_COUNTER:

        return "Step Counter (Non Wake-up)";
    case BHY_SID_STEP_COUNTER_WAKEUP:

        return "Step Counter (Wake-up)";
    case BHY_SID_GEOMAGNETIC_ROTATION_VECTOR:

        return "Geomagnetic Rotation (Non Wake-up)";
    case BHY_SID_GEOMAGNETIC_ROTATION_VECTOR_WAKEUP:

        return "Geomagnetic Rotation (Wake-up)";
    case BHY_SID_HEART_RATE:

        return "Heart Rate (Non Wake-up)";
    case BHY_SID_HEART_RATE_WAKEUP:

        return "Heart Rate (Wake-up)";
    case BHY_SID_TILT_DETECTOR:

        return "Tilt Detector (Non Wake-up)";
    case BHY_SID_TILT_DETECTOR_WAKEUP:

        return "Tilt Detector (Wake-up)";
    case BHY_SID_WAKE_GESTURE:

        return "Wake Gesture (Non Wake-up)";
    case BHY_SID_WAKE_GESTURE_WAKEUP:

        return "Wake Gesture (Wake-up)";
    case BHY_SID_GLANCE_GESTURE:

        return "Glance Gesture (Non Wake-up)";
    case BHY_SID_GLANCE_GESTURE_WAKEUP:

        return "Glance Gesture (Wake-up)";
    case BHY_SID_PICKUP_GESTURE:

        return "Pick-up Gesture (Non Wake-up)";
    case BHY_SID_PICKUP_GESTURE_WAKEUP:

        return "Pick-up Gesture (Wake-up)";
    case BHY_SID_CUS1:

        return "Pick-up Gesture (Non Wake-up)";
    case BHY_SID_CUS1_WAKEUP:

        return "Pick-up Gesture (Wake-up)";
    case BHY_SID_CUS2:

        return "Pick-up Gesture (Non Wake-up)";
    case BHY_SID_CUS2_WAKEUP:

        return "Pick-up Gesture (Wake-up)";
    case BHY_SID_CUS3:

        return "Pick-up Gesture (Non Wake-up)";
    case BHY_SID_CUS3_WAKEUP:

        return "Pick-up Gesture (Wake-up)";
    case BHY_SID_CUS4:

        return "Pick-up Gesture (Non Wake-up)";
    case BHY_SID_CUS4_WAKEUP:

        return "Pick-up Gesture (Wake-up)";
    case BHY_SID_CUS5:

        return "Pick-up Gesture (Non Wake-up)";
    case BHY_SID_CUS5_WAKEUP:

        return "Pick-up Gesture (Wake-up)";
    case BHY_SID_ACTIVITY:

        return "Activity (Non Wake-up)";
    case BHY_SID_ACTIVITY_WAKEUP:

        return "Activity (Wake-up)";
    case BHY_SID_DEBUG:

        return "Debug";
    case BHY_SID_TIMESTAMP_LSW:

        return "Timestamp LSW (Non Wake-up)";
    case BHY_SID_TIMESTAMP_LSW_WAKEUP:

        return "Timestamp LSW (Wake-up)";
    case BHY_SID_TIMESTAMP_MSW:

        return "Timestamp MSW (Non Wake-up)";
    case BHY_SID_TIMESTAMP_MSW_WAKEUP:

        return "Timestamp MSW (Wake-up)";
    case BHY_SID_META_EVENT:

        return "(Non Wake-up)";
    case BHY_SID_META_EVENT_WAKEUP:

        return "(Wake-up)";
    case BHY_SID_BSX_C:

        return "C";
    case BHY_SID_BSX_B:

        return "B";
    case BHY_SID_BSX_A:

        return "A";
    default:

        return "Unknown";
    }
}

String BHYSensor::getErrorString(int8_t errorStatus)
{
    String errorString;

    switch (errorStatus)
    {
    case BHY_E_NULL_PTR:
        errorString = "Null pointer";
        break;
    case BHY_E_COM_FAIL:
        errorString = "Communication failure";
        break;
    case BHY_E_DEV_NOT_FOUND:
        errorString = "Device not found";
        break;
    case BHY_E_OUT_OF_RANGE:
        errorString = "Out of range";
        break;
    case BHY_E_INVALID_INPUT:
        errorString = "Invalid input";
        break;
    case BHY_E_ACCEL_ODR_BW_INVALID:
        errorString = "Invalid accelerometer ODR BW setting";
        break;
    case BHY_E_GYRO_ODR_BW_INVALID:
        errorString = "Invalid gyroscope ODR BW setting";
        break;
    case BHY_E_LWP_PRE_FLTR_INT_INVALID:
        errorString = "LWP prefilter interrupt invalid";
        break;
    case BHY_E_LWP_PRE_FLTR_INVALID:
        errorString = "LWP prefilter invalid";
        break;
    case BHY_E_AUX_NOT_FOUND:
        errorString = "Auxiliary imu not found";
        break;
    case BHY_FOC_FAILURE:
        errorString = "FOC Failure";
        break;
    case BHY_E_INVALID_FIRMWARE:
        errorString = "Invalid Firmware";
        break;
    case BHY_E_RAMPATCH_MISMATCH:
        errorString = "RAM-Patch Mismatch";
        break;
    case BHY_E_CRC_MISMATCH:
        errorString = "CRC Mismatch";
        break;
    case BHY_E_PRODUCT_ID_MISMATCH:
        errorString = "Product ID Mismatch";
        break;
    case BHY_E_UNKNOWN_ERROR:
    default:
        errorString = "Unknown error";
        break;
    }

    return errorString;
}

String BHYSensor::getMetaEventName(bhyMetaEventType type)
{
    String metaName;

    switch (type)
    {
    case BHY_META_TYPE_NOT_USED:
        metaName = "Unused";
        break;
    case BHY_META_TYPE_FLUSH_COMPLETE:
        metaName = "Flush complete";
        break;
    case BHY_META_TYPE_SAMPLE_RATE_CHANGED:
        metaName = "Sample rate changed";
        break;
    case BHY_META_TYPE_POWER_MODE_CHANGED:
        metaName = "Power mode changed";
        break;
    case BHY_META_TYPE_ERROR:
        metaName = "Error";
        break;
    case BHY_META_TYPE_ALGORITHM:
        metaName = "Algorithm";
        break;
    case BHY_META_TYPE_SENSOR_ERROR:
        metaName = "Sensor error";
        break;
    case BHY_META_TYPE_FIFO_OVERFLOW:
        metaName = "FIFO overflow";
        break;
    case BHY_META_TYPE_DYNAMIC_RANGE_CHANGED:
        metaName = "Dynamic range changed";
        break;
    case BHY_META_TYPE_FIFO_WATERMARK:
        metaName = "FIFO watermark";
        break;
    case BHY_META_TYPE_SELF_TEST_RESULTS:
        metaName = "Self test results";
        break;
    case BHY_META_TYPE_INITIALIZED:
        metaName = "Device initialized";
        break;
    default:
        metaName = "Invalid";
        break;
    }

    return metaName;
}
