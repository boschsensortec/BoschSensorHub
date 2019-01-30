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
 * @file    bhy_defs_internal.h
 * @date    30.1.2019
 * @version 0.1.0
 *
 */

/**************************************************************/
/**\name    I2C REGISTER MAP DEFINITIONS                      */
/**************************************************************/
// Read buffer constants
#define BHY_REG_BUFFER_ZERO_ADDR            UINT8_C(0x00)
#define BHY_REG_BUFFER_END_ADDR             UINT8_C(0x31)
#define BHY_REG_BUFFER_LENGTH               UINT8_C(0x32)

// FIFO flush, chip control and status registers
#define BHY_REG_FIFO_FLUSH_ADDR             UINT8_C(0x32)
#define BHY_REG_CHIP_CONTROL_ADDR           UINT8_C(0x34)
#define BHY_REG_HOST_STATUS_ADDR            UINT8_C(0x35)
#define BHY_REG_INT_STATUS_ADDR             UINT8_C(0x36)
#define BHY_REG_CHIP_STATUS_ADDR            UINT8_C(0x37)

// bytes remaining register
#define BHY_REG_BYTES_REMAINING_ADDR        UINT8_C(0x38)
#define BHY_REG_BYTES_REMAINING_MSB_ADDR    UINT8_C(0x39)
#define BHY_REG_PARAMETER_ACKNOWLEDGE_ADDR  UINT8_C(0x3A)

// saved parameter
#define BHY_REG_PARAMETER_READ_BUFFER_ZERO  UINT8_C(0x3B)

#define BHY_REG_PARAMETER_PAGE_SELECT_ADDR  UINT8_C(0x54)

// parameter page selection address
#define BHY_REG_HOST_INTERFACE_CONTROL_ADDR UINT8_C(0x55)

#define BHY_REG_PARAMETER_WRITE_BUFFER_ZERO UINT8_C(0x5C)
#define BHY_REG_PARAMETER_REQUEST_ADDR      UINT8_C(0x64)

// product and revision id
#define BHY_IRQ_TIMESTAMP_ADDR              UINT8_C(0x6C)
#define BHY_ROM_VERSION_ADDR                UINT8_C(0x70)
#define BHY_RAM_VERSION_ADDR                UINT8_C(0x72)
#define BHY_REG_PRODUCT_ID_ADDR             UINT8_C(0x90)
#define BHY_REG_REVISION_ID_ADDR            UINT8_C(0x91)
#define BHY_REG_UPLOAD_0_ADDR               UINT8_C(0x94)
#define BHY_REG_UPLOAD_1_ADDR               UINT8_C(0x95)
#define BHY_REG_UPLOAD_DATA_ADDR            UINT8_C(0x96)
#define BHY_REG_CRC_HOST_ADDR               UINT8_C(0x97)
#define BHY_REG_RESET_REQUEST_ADDR          UINT8_C(0x9B)

// Same?
#define BHY_CRC_HOST_SIZE                   UINT8_C(4)
#define BHY_CRC_HOST_LENGTH                 UINT8_C(4)

#define BHY_CRC_HOST_LSB                    UINT8_C(0)
#define BHY_CRC_HOST_XLSB                   UINT8_C(1)
#define BHY_CRC_HOST_XXLSB                  UINT8_C(2)
#define BHY_CRC_HOST_MSB                    UINT8_C(3)
#define BHY_CRC_HOST_FILE_LSB               UINT8_C(4)
#define BHY_CRC_HOST_FILE_XLSB              UINT8_C(5)
#define BHY_CRC_HOST_FILE_XXLSB             UINT8_C(6)
#define BHY_CRC_HOST_FILE_MSB               UINT8_C(7)

/**************************************************************/
/**\name PARAMETER PAGE SELECTION SELECTION DEFINITIONS       */
/**************************************************************/
#define BHY_PAGE_SELECT_PARAMETER_PAGE      (0)
#define BHY_PAGE_SELECT_PARAMETER_SIZE      (1)

#define BHY_META_EVENT_1_INT_ENABLE_BIT     (1 << 0)
#define BHY_META_EVENT_1_ENABLE_BIT         (1 << 1)

#define BHY_MASK_LSB_DATA                   UINT16_C(0x00FF)
#define BHY_MASK_MSB_DATA                   UINT16_C(0xFF00)
#define BHY_MASK_LSW_TIMESTAMP              UINT32_C(0x0000FFFF)
#define BHY_MASK_MSW_TIMESTAMP              UINT32_C(0xFFFF0000)

#define BHY_SENSOR_PARAMETER_WRITE          UINT8_C(0xC0)

#define BHY_ACK_DELAY                       UINT8_C(50)

#define BHY_CHIP_CTRL_HOST_UPLOAD_BIT       UINT8_C(0x02)
#define BHY_CHIP_CTRL_CPU_RUN_BIT           UINT8_C(0x01)

#define BHY_SIGNATURE_MEM_LEN               UINT8_C(17)

#define BHY_RAM_WRITE_LENGTH                UINT8_C(4)
#define BHY_RAM_WRITE_LENGTH_API            UINT8_C(16)

#define BHY_SIGNATURE_1                     UINT8_C(0)
#define BHY_SIGNATURE_2                     UINT8_C(1)
#define BHY_IMAGE_SIGNATURE1                UINT8_C(0x2A)
#define BHY_IMAGE_SIGNATURE2                UINT8_C(0x65)
#define BHY_SIG_FLAG_1_POS                  UINT8_C(2)
#define BHY_SIG_FLAG_2_POS                  UINT8_C(3)
