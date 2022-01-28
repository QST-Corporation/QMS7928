/**************************************************************************************************
 
  Shanghai QST Corporation confidential and proprietary. 
  All rights reserved.

  IMPORTANT: All rights of this software belong to Shanghai QST 
  Corporation ("QST"). Your use of this Software is limited to those 
  specific rights granted under  the terms of the business contract, the 
  confidential agreement, the non-disclosure agreement and any other forms 
  of agreements as a customer or a partner of QST. You may not use this 
  Software unless you agree to abide by the terms of these agreements. 
  You acknowledge that the Software may not be modified, copied, 
  distributed or disclosed unless embedded on a QST Bluetooth Low Energy 
  (BLE) integrated circuit, either as a product or is integrated into your 
  products.  Other than for the aforementioned purposes, you may not use, 
  reproduce, copy, prepare derivative works of, modify, distribute, perform, 
  display or sell this Software and/or its documentation for any purposes.

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
  PROVIDED AS IS WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
  INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
  NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
  QST OR ITS SUBSIDIARIES BE LIABLE OR OBLIGATED UNDER CONTRACT,
  NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
  LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
  INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
  OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
  OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
  (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
  
*******************************************************************************/

/******************************************************************************
 * @file    qma6100.h
 * @author  QST AE team
 * @version V0.1
 * @date    2022-01-06
 * @id      $Id$
 * @brief   This file provides the functions for QST QMA6100 sensor evaluation.
 *
 * @note
 *
 *****************************************************************************/

#ifndef _QMA6100_H_
#define _QMA6100_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include "i2c.h"
#include "gpio.h"

/******************************************************
 *                      Macros
 ******************************************************/
#define HPF 0x40
#define LPF 0x00
/******************************************************
 *                    Constants
 ******************************************************/

/******************************************************
 *                   Enumerations
 ******************************************************/
typedef enum{
    QMA_SUCCESS,
    QMA_ERROR,
}ret_code_t;
typedef enum{
    if_i2c  = 0x00,
    if_spi_polling  = 0x01,
    if_spi_int = 0x02,
    if_spi_dma = 0x03,
}qma6100_if_type_t;

typedef enum{
    i2c_speed_100K = I2C_CLOCK_100K,
    i2C_speed_400K = I2C_CLOCK_400K,
    spi_speed_1M = 1000000,
    spi_speed_2M = 2000000,
    spi_speed_4M = 4000000,
    spi_speed_8M = 8000000,
}if_speed_t;

typedef enum{
    MCLK_512KHZ,
    MCLK_307KHZ,
    MCLK_205KHZ,
    MCLK_100KHZ,
    MCLK_51KHZ,
    MCLK_25KHZ,
    MCLK_12KHZ,
    MCLK_6KHZ,
    MCLK_Reserved
}qma6100_mclk_t;

typedef enum{
    STANDBYMODE,
    WAKEMODE,
}qma6100_mode_t;

typedef enum
{
    RANGE_2G = 0x1,
    RANGE_4G = 0x2,
    RANGE_8G = 0x4,
    RANGE_16G = 0x8,
    RANGE_32G = 0xf
}qma6100_range_t;

typedef enum
{
    AM_SLOPE=0x00,
    AM_GRAVITY=0x40
}am_type_t;

typedef enum
{
    HPCF_NONE=0x00,
    HPCF_ODRDIV10=(0x10<<1),
    HPCF_ODRDIV25=0x20<<1,
    HPCF_ODRDIV50=0x30<<1,
    HPCF_ODRDIV100=0x40<<1,
    HPCF_ODRDIV200=0x50<<1,
    HPCF_ODRDIV400=0x60<<1,
    HPCF_ODRDIV800=0x70<<1
}hpfcf_t;

typedef enum
{
    LPCF_NONE=0x00<<1,
    LPCF_AVG1=0x40<<1,
    LPCF_AVG2=0x10<<1,
    LPCF_AVG3=0x20<<1,
    LPCF_AVG4=0x30<<1
}lpfcf_t;

typedef enum
{
	QMA6100_TAP_SINGLE = 0x80,
	QMA6100_TAP_DOUBLE = 0x20,
	QMA6100_TAP_TRIPLE = 0x10,
	QMA6100_TAP_QUARTER = 0x01,
	QMA6100_TAP_NONE = 0x00
}qma6100P_tap_t;

typedef enum
{
    AXIS_X=0x01,
    AXIS_Y=0x02,
    AXIS_Z=0x04
}axis_sel_t;

typedef enum
{
    PORT_1=1,
    PORT_2=2,
    PORT_3=3,
}int_port_t;

typedef enum
{
    FIFO_MODE_FIFO     = 0x40,
    FIFO_MODE_STREAM   = 0x80,
    FIFO_MODE_BYPASS   = 0x00,
    FIFO_INT_WATERMARK = 0x40,
    FIFO_INT_FULL      = 0x20,
    FIFO_INT_NONE      = 0xFF
}qma6100_fifo_t;

/******************************************************
 *                    Structures
 ******************************************************/
typedef struct 
{
    float x;
    float y;
    float z;
}three_axis_t;

typedef union
{
    float        data[3];
    three_axis_t axis;
}qma6100_raw_t;

typedef ret_code_t (*qma6100_reg_read_byte_t) (uint8_t reg, uint8_t * pData);
typedef ret_code_t (*qma6100_reg_read_multi_byte_t)(uint8_t reg, uint8_t * pData, uint8_t size);
typedef ret_code_t (*qma6100_reg_write_byte_t) (uint8_t reg, uint8_t val);

typedef struct{
    gpio_pin_e scl;
    gpio_pin_e sda;
    gpio_pin_e cs;
    gpio_pin_e sck;
    gpio_pin_e mosi;
    gpio_pin_e miso;
    gpio_pin_e int1;
    gpio_pin_e int2;
}if_pin_t;

typedef struct
{
    qma6100_if_type_t type;
    if_speed_t speed;
    uint8_t i2c_address;
    if_pin_t pin;
    qma6100_reg_read_byte_t read_byte;
    qma6100_reg_read_multi_byte_t read_multi_byte;
    qma6100_reg_write_byte_t write_byte;
}qma6100_if_handle_t;

typedef struct
{
    bool initialized;
    uint8_t dev_id;
    //uint8_t revision;
    //qma6100_config_t cfg;
    qma6100_raw_t raw;
    const qma6100_if_handle_t *hw_if;
}qma6100_device_t;

/******************************************************
 *                 Global Variables
 ******************************************************/

/******************************************************
 *             Function Declarations
 ******************************************************/
qma6100_device_t* get_qma6100_handle(void);
void qma6100_demo(void);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* _QMA6100_H_ */
