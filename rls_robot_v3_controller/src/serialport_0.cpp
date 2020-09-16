/************************************************************************************//**
* \file         port/linux/serialport_0.c
* \brief        Serial port source file.
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

/****************************************************************************************
* Include files
****************************************************************************************/
#include <assert.h>                         /* for assertions                          */
#include <stdint.h>                         /* for standard integer types              */
#include <stddef.h>                         /* for NULL declaration                    */
#include <stdbool.h>                        /* for boolean type                        */
#include <unistd.h>                         /* UNIX standard functions                 */
#include <termios.h>                        /* POSIX terminal control                  */
#include <fcntl.h>                          /* file control definitions                */
#include <sys/ioctl.h>                      /* system I/O control                      */
#include "../include/rls_robot_v3_controller/serialport_0.h"                     /* serial port module                      */


/****************************************************************************************
* Macro definitions
****************************************************************************************/
/** \brief Invalid serial port device handle. */
#define SERIALPORT_INVALID_HANDLE_0      (-1)


/****************************************************************************************
* Local data declarations
****************************************************************************************/
/** \brief Serial port handle. */
static int32_t portHandle_0 = -1;


/****************************************************************************************
* Local constant declarations
****************************************************************************************/
/** \brief Lookup table for converting this module's generic baudrate value to a value
 *         supported by the low level interface.
 */
static const speed_t baudrateLookup_0[] =
{
  B9600,                                    /**< Index 0 = SERIALPORT_BR9600           */
  B19200,                                   /**< Index 1 = SERIALPORT_BR19200          */
  B38400,                                   /**< Index 2 = SERIALPORT_BR38400          */
  B57600,                                   /**< Index 3 = SERIALPORT_BR57600          */
  B115200                                   /**< Index 4 = SERIALPORT_BR115200         */
};


/************************************************************************************//**
** \brief     Closes the connection with the serial port.
**
****************************************************************************************/
void SerialPortClose_0(void)
{
  /* Close the port handle if valid. */
  if (portHandle_0 != SERIALPORT_INVALID_HANDLE_0)
  {
    close(portHandle_0);
  }
  /* Invalidate handle. */
  portHandle_0 = SERIALPORT_INVALID_HANDLE_0;
} /*** end of SerialPortClose_0 ***/


/************************************************************************************//**
** \brief     Opens the connection with the serial port configured as 8,N,1 and no flow
**            control.
** \param     portname The name of the serial port to open, i.e. /dev/ttyUSB0.
** \param     baudrate The desired communication speed.
** \return    True if successful, false otherwise.
**
****************************************************************************************/
bool SerialPortOpen_0(char const * portname, tSerialPortBaudrate_0 baudrate) 
{
  bool result = false;
  struct termios options = { 0 }; 
  int32_t iFlags;

  /* Check parameters. */
  assert(portname != NULL);
  
  /* Only continue with valid parameters. */
  if (portname != NULL) /*lint !e774 */
  {
    /* Assume the result to be okay from here on and only set it to error when a problem
     * was detected.
     */
    result = true;
    /* Open the port. */
    portHandle_0 = open(portname, O_RDWR | O_NOCTTY | O_NDELAY);
    /* Check the result */
    if (portHandle_0 == SERIALPORT_INVALID_HANDLE_0)
    {
      result = false;
    }
    
    /* Configure the device to block during read operations. */
    if (result)
    {
      if (fcntl(portHandle_0, F_SETFL, 0) == -1)
      {
        SerialPortClose_0();
        result = false;
      }
    }
    /* Get the current options for the port. */
    if (result)
    {
      if (tcgetattr(portHandle_0, &options) == -1)
      {
        SerialPortClose_0();
        result = false;
      }
    }
    /* Configure the baudrate. */
    if (result)
    {
      if (cfsetispeed(&options, baudrateLookup_0[baudrate]) == -1)
      {
        SerialPortClose_0();
        result = false;
      }
    }
    if (result)
    {
      if (cfsetospeed(&options, baudrateLookup_0[baudrate]) == -1)
      {
        SerialPortClose_0();
        result = false;
      }
    }

    if (result)
    {
      /* Input modes - clear indicated ones giving: no break, no CR to NL,
       * no parity check, no strip char, no start/stop output (sic) control.
       */
      options.c_iflag &= ~(BRKINT | ICRNL | INPCK | ISTRIP | IXON);
      /* Output modes - clear giving: no post processing such as NL to CR+NL. */
      options.c_oflag &= ~(OPOST);
      /* Control modes - set 8 bit chars */
      options.c_cflag |= (CS8);
      /* Local modes - clear giving: echoing off, canonical off (no erase with
       * backspace, ^U,...),  no extended functions, no signal chars (^Z,^C).
       */
      options.c_lflag &= ~(ECHO | ICANON | IEXTEN | ISIG);
      /* Configure timeouts. */
      options.c_cc[VMIN]  = 0;
      options.c_cc[VTIME] = 1; /* in units of 1/10th of a second */
      /* Set the new options for the port. */
      if (tcsetattr(portHandle_0, TCSAFLUSH, &options) == -1)
      {
        SerialPortClose_0();
        result = false;
      }
    }
    /* Turn on DTR. */
    if (result)
    {
      iFlags = TIOCM_DTR;
      if (ioctl(portHandle_0, TIOCMBIS, &iFlags) == -1)
      {
        SerialPortClose_0();
        result = false;
      }
    }
  }
  
  /* Give the result back to the caller. */
  return result;
} /*** end of SerialPortOpen ***/


/************************************************************************************//**
** \brief     Writes data to the serial port.
** \param     data Pointer to byte array with data to write.
** \param     length Number of bytes to write.
** \return    True if successful, false otherwise.
**
****************************************************************************************/
bool SerialPortWrite_0(uint8_t const * data, uint32_t length)
{
  bool result = false;

  /* Check parameters. */
  assert(data != NULL);
  assert(length > 0);

  /* Only continue with valid parameters. */
  if ( (data != NULL) && (length > 0) ) /*lint !e774 */
  {
    /* Submit the data for sending. */
    if ((uint32_t)write(portHandle_0, data, length) == length)
    {
      result = true;
    }
  }
  /* Give the result back to the caller. */
  return result;
} /*** end of SerialPortWrite ***/


/*********************************** end of serialport_0.c *******************************/

