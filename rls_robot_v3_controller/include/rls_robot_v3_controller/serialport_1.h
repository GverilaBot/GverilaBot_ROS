/************************************************************************************//**
* \file         serialport_1.h
* \brief        Serial port header file.
* \ingroup      SerialPort
* \internal
*----------------------------------------------------------------------------------------
*                          C O P Y R I G H T
*----------------------------------------------------------------------------------------
*   Copyright (c) 2017  by Feaser    http://www.feaser.com    All rights reserved
*
*----------------------------------------------------------------------------------------
*                            L I C E N S E
*----------------------------------------------------------------------------------------
* This file is part of OpenBLT. OpenBLT is free software: you can redistribute it and/or
* modify it under the terms of the GNU General Public License as published by the Free
* Software Foundation, either version 3 of the License, or (at your option) any later
* version.
*
* OpenBLT is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
* without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
* PURPOSE. See the GNU General Public License for more details.
*
* You have received a copy of the GNU General Public License along with OpenBLT. It
* should be located in ".\Doc\license.html". If not, contact Feaser to obtain a copy.
*
* \endinternal
****************************************************************************************/
/************************************************************************************//**
* \defgroup   SerialPort Serial port driver
* \brief      This module implements a generic serial port driver.
* \ingroup    Session
****************************************************************************************/
#ifndef SERIALPORT_1_H
#define SERIALPORT_1_H

#ifdef __cplusplus
extern "C" {
#endif

/****************************************************************************************
* Type definitions
****************************************************************************************/
/** \brief Enumeration of the supported baudrates. */
typedef enum
{
  SERIALPORT_1_BR9600   = 0,                  /**< 9600 bits/sec                         */
  SERIALPORT_1_BR19200  = 1,                  /**< 19200 bits/sec                        */
  SERIALPORT_1_BR38400  = 2,                  /**< 38400 bits/sec                        */
  SERIALPORT_1_BR57600  = 3,                  /**< 57600 bits/sec                        */
  SERIALPORT_1_BR115200 = 4                   /**< 115200 bits/sec                       */
} tSerialPortBaudrate_1;


/****************************************************************************************
* Function prototypes
****************************************************************************************/
bool SerialPortOpen_1(char const * portname, tSerialPortBaudrate_1 baudrate);
void SerialPortClose_1(void);
bool SerialPortWrite_1(uint8_t const * data, uint32_t length);

#ifdef __cplusplus
}
#endif

#endif /* SERIALPORT_1_H */
/********************************* end of serialport_1.h *********************************/

