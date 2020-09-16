/************************************************************************************//**
* \file         ODrive.h
* \brief        ODrive library header file.
* \ingroup      Library
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
* \defgroup   Library Library API
* \brief      ODrive Library API.
* \details
* The Library API contains the application programming interface for the ODrive libary.
* It defines the functions and definitions that an external program uses to access the
* library's functionality.
****************************************************************************************/
#ifndef ODRIVE_H_1
#define ODRIVE_H_1

#ifdef __cplusplus
extern "C" {
#endif

/****************************************************************************************
* Include files
****************************************************************************************/
#include <stdint.h>                         /* for standard integer types              */


/****************************************************************************************
* Macro definitions
****************************************************************************************/
/* CMake automatically defines macro ODrive_shared_EXPORTS when building the shared
 * version of the library. When building under windows, this is used to set the export
 * macro, which is needed to confige the library functions.
 */
#if defined(_WIN32) || defined(_WIN64)
#if defined(ODrive_shared_EXPORTS)
#define  LIBODRIVE_EXPORT __declspec(dllexport)
#else
#define  LIBODRIVE_EXPORT 
#endif /* ODrive_shared_EXPORTS */
#else /* defined(_WIN32) || defined(_WIN64) */
#define LIBODRIVE_EXPORT
#endif

#define LIBINSTANCE _1

#if !defined(LIBINSTANCE)
  #define SET_FUNC_NAME(F) F
#else
  #define SET_FUNC_NAME2(F, N) F ## N
  #define SET_FUNC_NAME1(F, N) SET_FUNC_NAME2(F, N)
  #define SET_FUNC_NAME(F) SET_FUNC_NAME1(F, LIBINSTANCE)
#endif

/** \brief Function return value for when everything went okay. */
#define LIBODRIVE_RESULT_OK                  (0u)

/** \brief Function return value for when a generic error occured. */
#define LIBODRIVE_RESULT_ERROR_GENERIC       (1u)


/****************************************************************************************
*             V E R S I O N   I N F O R M A T I O N
****************************************************************************************/
/****************************************************************************************
* Function prototypes
****************************************************************************************/
LIBODRIVE_EXPORT uint32_t SET_FUNC_NAME(LibODriveVersionGetNumber)(void);
LIBODRIVE_EXPORT char const * SET_FUNC_NAME(LibODriveVersionGetString)(void);


/****************************************************************************************
*             S E S S I O N   /   T R A N S P O R T   L A Y E R S
****************************************************************************************/
/****************************************************************************************
* Macro definitions
****************************************************************************************/
/** \brief  */
#define LIBODRIVE_SESSION_ASCII              ((uint32_t)0u)

/** \brief Transport layer that uses RS-232 serial communication for data exchange. */
#define LIBODRIVE_TRANSPORT_RS232            ((uint32_t)0u)

/** \brief Transport layer that uses Controller Area Network (CAN) for data exchange. */
#define LIBODRIVE_TRANSPORT_CAN              ((uint32_t)1u)

/** \brief Transport layer that uses USB Bulk for data exchange. */
#define LIBODRIVE_TRANSPORT_USB              ((uint32_t)2u)

/** \brief Transport layer that uses TCP/IP for data exchange. */
#define LIBODRIVE_TRANSPORT_NET              ((uint32_t)3u)


/****************************************************************************************
* Type definitions
****************************************************************************************/
/** \brief Structure layout of the Terminal session settings. */
typedef struct t_libODrive_session_settings_ascii
{
  uint16_t timeout;              /**< Command response timeout in milliseconds.        */
} tLibODriveSessionSettingsAscii;

/** \brief Structure layout of the RS232 transport layer settings. The portName field is
 *         platform dependent. On Linux based systems this should be the filename of the
 *         tty-device, such as "/dev/tty0". On Windows based systems it should be the
 *         name of the COM-port, such as "COM1".
 */
typedef struct t_libODrive_transport_settings_rs232
{
  char const * portName;         /**< Communication port name such as /dev/tty0.       */
  uint32_t baudrate;             /**< Communication speed in bits/sec.                 */
} tLibODriveTransportSettingsRs232;

/** \brief Structure layout of the CAN transport layer settings. The deviceName field is
 *         platform dependent.
 *         On Linux based systems this should be the socketCAN
 *         interface name such as "can0". The terminal command "ip addr" can be issued to
 *         view a list of interfaces that are up and available. Under Linux it is assumed
 *         that the socketCAN interface is already configured on the system, before using
 *         the OpenBLT library. When baudrate is configured when bringing up the system,
 *         so the baudrate field in this structure is don't care when using the library
 *         on a Linux was system.
 *         On Windows based systems, the device name is a name that is pre-defined by
 *         this library for the supported CAN adapters. The device name should be one of
 *         the following: "peak_pcanusb", "kvaser_leaflight", or "lawicel_canusb".
 *         Field use extended is a boolean field. When set to 0, the specified transmitId
 *         and receiveId are assumed to be 11-bit standard CAN identifier. It the field
 *         is 1, these identifiers are assumed to be 29-bit extended CAN identifiers.
 */
typedef struct t_libODrive_transport_settings_can
{
  char const * deviceName;       /**< Device name such as can0, peak_pcanusb etc.      */
  uint32_t deviceChannel;        /**< Channel on the device to use.                    */
  uint32_t baudrate;             /**< Communication speed in bits/sec.                 */
  uint32_t transmitId;           /**< Transmit CAN identifier.                         */
  uint32_t receiveId;            /**< Receive CAN identifier.                          */
  uint32_t useExtended;          /**< Boolean to configure 29-bit CAN identifiers.     */
} tLibODriveTransportSettingsCan;

/** \brief Structure layout of the NET transport layer settings. The address field can be
 *         set to either the IP address or the hostname, such as "192.168.178.23" or
 *         "mymicro.mydomain.com". The port should be set to the  TCP port number that
 *         the bootloader target listens on.
 */
typedef struct t_libODrive_transport_settings_net
{
  char const * address;          /**< Target IP-address or hostname on the network.    */
  uint16_t port;                 /**< TCP port to use.                                 */
} tLibODriveTransportSettingsNet;


/****************************************************************************************
* Function prototypes
****************************************************************************************/
LIBODRIVE_EXPORT void SET_FUNC_NAME(LibODriveSessionInit)(uint32_t sessionType,
                                           void const * sessionSettings,
                                           uint32_t transportType, 
                                           void const * transportSettings);
LIBODRIVE_EXPORT void SET_FUNC_NAME(LibODriveSessionTerminate)(void);
LIBODRIVE_EXPORT uint32_t SET_FUNC_NAME(LibODriveSessionStart)(void);
LIBODRIVE_EXPORT void SET_FUNC_NAME(LibODriveSessionStop)(void);
LIBODRIVE_EXPORT uint32_t SET_FUNC_NAME(LibODriveSessionWriteData)(uint8_t const * txData,
                                                    uint32_t txLen,
                                                    uint8_t * rxData, uint32_t * rxLen);
LIBODRIVE_EXPORT uint32_t SET_FUNC_NAME(LibODriveSessionReadData)(uint8_t * data, uint32_t len);


/****************************************************************************************
*             G E N E R I C   U T I L I T I E S
****************************************************************************************/
/****************************************************************************************
* Function prototypes
****************************************************************************************/
LIBODRIVE_EXPORT uint16_t SET_FUNC_NAME(LibODriveUtilCrc16Calculate)(uint8_t const * data,
                                                      uint32_t len);
LIBODRIVE_EXPORT uint32_t SET_FUNC_NAME(LibODriveUtilCrc32Calculate)(uint8_t const * data,
                                                      uint32_t len);
LIBODRIVE_EXPORT uint32_t SET_FUNC_NAME(LibODriveUtilTimeGetSystemTime)(void);
LIBODRIVE_EXPORT void SET_FUNC_NAME(LibODriveUtilTimeDelayMs)(uint16_t delay);
LIBODRIVE_EXPORT uint32_t SET_FUNC_NAME(LibODriveUtilCryptoAes256Encrypt)(uint8_t * data, uint32_t len,
                                                           uint8_t const * key);
LIBODRIVE_EXPORT uint32_t SET_FUNC_NAME(LibODriveUtilCryptoAes256Decrypt)(uint8_t * data, uint32_t len,
                                                           uint8_t const * key);


#ifdef __cplusplus
}
#endif

#endif /* ODRIVE_H */
/********************************** end of ODrive.h ************************************/
