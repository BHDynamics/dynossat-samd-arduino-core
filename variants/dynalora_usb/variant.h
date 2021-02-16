/*
  Copyright (c) 2014-2015 Arduino LLC.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  See the GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#ifndef _VARIANT_DYNALORA_USB_
#define _VARIANT_DYNALORA_USB_

// The definitions here needs a SAMD core >=1.6.10
#define ARDUINO_SAMD_VARIANT_COMPLIANCE 10610

/*----------------------------------------------------------------------------
 *        Definitions
 *----------------------------------------------------------------------------*/

/** Frequency of the board main oscillator */
#define VARIANT_MAINOSC		(32768ul)

/** Master clock frequency */
#define VARIANT_MCK	(F_CPU)

/*----------------------------------------------------------------------------
 *        Headers
 *----------------------------------------------------------------------------*/

#include "WVariant.h"

#ifdef __cplusplus
#include "SERCOM.h"
#include "Uart.h"
#endif // __cplusplus

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

/*----------------------------------------------------------------------------
 *        Pins
 *----------------------------------------------------------------------------*/

// Number of pins defined in PinDescription array
#define PINS_COUNT           (22u)
#define NUM_DIGITAL_PINS     (21u)
#define NUM_ANALOG_INPUTS    (1u)
#define NUM_ANALOG_OUTPUTS   (1u)
#define analogInputToDigitalPin(p)  ((p < 1u) ? (p) + (PIN_A0) : -1)

#define digitalPinToPort(P)        ( &(PORT->Group[g_APinDescription[P].ulPort]) )
#define digitalPinToBitMask(P)     ( 1 << g_APinDescription[P].ulPin )
//#define analogInPinToBit(P)        ( )
#define portOutputRegister(port)   ( &(port->OUT.reg) )
#define portInputRegister(port)    ( &(port->IN.reg) )
#define portModeRegister(port)     ( &(port->DIR.reg) )
#define digitalPinHasPWM(P)        ( g_APinDescription[P].ulPWMChannel != NOT_ON_PWM || g_APinDescription[P].ulTCChannel != NOT_ON_TIMER )

/*
 * digitalPinToTimer(..) is AVR-specific and is not defined for SAMD
 * architecture. If you need to check if a pin supports PWM you must
 * use digitalPinHasPWM(..).
 *
 * https://github.com/arduino/Arduino/issues/1833
 */
// #define digitalPinToTimer(P)

// LEDs
#define PIN_LED_13           (10u)
#define PIN_LED              PIN_LED_13
#define LED_BUILTIN          PIN_LED_13
#define NEOPIXEL_BUILTIN     (11u)
#define PIN_NEOPIXEL		 NEOPIXEL_BUILTIN

static const uint8_t D13  = PIN_LED_13;

/*
 * Analog pins
 */
#define PIN_A0               (2u)

static const uint8_t A0  = PIN_A0;

#define PIN_DAC0			PIN_A0

#define ADC_RESOLUTION		12

/*
 * Digital pins
 */
#define PIN_D0               (0ul)
#define PIN_D1               (PIN_D0 + 1)
#define PIN_D2               (PIN_D0 + 2)
#define PIN_D3               (PIN_D0 + 3)
#define PIN_D4               (PIN_D0 + 4)
#define PIN_D5               (PIN_D0 + 5)
#define PIN_D6               (PIN_D0 + 6)
#define PIN_D7               (PIN_D0 + 7)
#define PIN_D8               (PIN_D0 + 8)
#define PIN_D9               (PIN_D0 + 9)

static const uint8_t D0  = PIN_D0;
static const uint8_t D1  = PIN_D1;
static const uint8_t D2  = PIN_D2;
static const uint8_t D3  = PIN_D3;
static const uint8_t D4  = PIN_D4;
static const uint8_t D5  = PIN_D4;
static const uint8_t D6  = PIN_D5;
static const uint8_t D7  = PIN_D7;
static const uint8_t D8  = PIN_D8;
static const uint8_t D9  = PIN_D9;

static const uint8_t PA00 = PIN_D0;
static const uint8_t PA01 = PIN_D1;
static const uint8_t PA02 = PIN_D2;
static const uint8_t SWCLK = PIN_D3;
static const uint8_t SWDIO = PIN_D4;

static const uint8_t BUTTON = PIN_D5;
static const uint8_t RADIO_INT = PIN_D6;
static const uint8_t RADIO_RESET = PIN_D7;
static const uint8_t RADIO_CS = PIN_D8;
static const uint8_t SD_CS = PIN_D9;

// On-board SPI Flash
#define EXTERNAL_FLASH_DEVICES  GD25Q32C
#define EXTERNAL_FLASH_USE_SPI  SPI1
#define EXTERNAL_FLASH_USE_CS   SS1

/*
 * Serial interfaces
 */

// Serial1 - SERCOM1
#define PIN_SERIAL1_RX       PIN_D1
#define PIN_SERIAL1_TX       PIN_D0
#define PAD_SERIAL1_TX       (UART_TX_PAD_0)
#define PAD_SERIAL1_RX       (SERCOM_RX_PAD_1)

static const uint8_t TX = PIN_SERIAL1_TX;
static const uint8_t RX = PIN_SERIAL1_RX;

/*
 * SPI Interfaces
 */
#define SPI_INTERFACES_COUNT 3

#define PIN_SPI_MISO         (12u)
#define PIN_SPI_MOSI         (13u)
#define PIN_SPI_SCK          (14u)
#define PERIPH_SPI           sercom3
#define PAD_SPI_TX           SPI_PAD_0_SCK_1
#define PAD_SPI_RX           SERCOM_RX_PAD_2

static const uint8_t SS	  = (-1) ;	// SERCOM4 last PAD is present on A2 but HW SS isn't used. Set here only for reference.
static const uint8_t MOSI = PIN_SPI_MOSI ;
static const uint8_t MISO = PIN_SPI_MISO ;
static const uint8_t SCK  = PIN_SPI_SCK ;

static const uint8_t SDO = PIN_SPI_MOSI ;
static const uint8_t SDI = PIN_SPI_MISO ;

// Flash memory SPI

#define PIN_SPI1_MISO         (19u)
#define PIN_SPI1_MOSI         (20u)
#define PIN_SPI1_SCK          (21u)
#define PERIPH_SPI1           sercom0
#define PAD_SPI1_TX           SPI_PAD_0_SCK_3
#define PAD_SPI1_RX           SERCOM_RX_PAD_1

static const uint8_t SS1   = 22 ;
static const uint8_t MOSI1 = PIN_SPI1_MOSI ;
static const uint8_t MISO1 = PIN_SPI1_MISO ;
static const uint8_t SCK1  = PIN_SPI1_SCK ;

static const uint8_t SDO1 = PIN_SPI1_MOSI ;
static const uint8_t SDI1 = PIN_SPI1_MISO ;

static const uint8_t FLASH_SDO = PIN_SPI1_MOSI ;
static const uint8_t FLASH_SDI = PIN_SPI1_MISO ;
static const uint8_t FLASH_SCK = PIN_SPI1_SCK ;
static const uint8_t FLASH_CS = SS1 ;

// External header SPI interface

#define PIN_SPI2_MISO         PIN_D3
#define PIN_SPI2_MOSI         PIN_D0
#define PIN_SPI2_SCK          PIN_D1
#define PERIPH_SPI2           sercom1
#define PAD_SPI2_TX           SPI_PAD_0_SCK_1
#define PAD_SPI2_RX           SERCOM_RX_PAD_2

static const uint8_t MOSI2 = PIN_SPI2_MOSI ;
static const uint8_t MISO2 = PIN_SPI2_MISO ;
static const uint8_t SCK2  = PIN_SPI2_SCK ;

static const uint8_t SDO2 = PIN_SPI2_MOSI ;
static const uint8_t SDI2 = PIN_SPI2_MISO ;

/*
 * Wire Interfaces
 */
#define WIRE_INTERFACES_COUNT 1

#define PIN_WIRE_SDA         (0u)
#define PIN_WIRE_SCL         (1u)
#define PERIPH_WIRE          sercom1
#define WIRE_IT_HANDLER      SERCOM1_Handler

static const uint8_t SDA = PIN_WIRE_SDA;
static const uint8_t SCL = PIN_WIRE_SCL;

/*
 * USB
 */
#define PIN_USB_HOST_ENABLE (16ul)
#define PIN_USB_DM          (17ul)
#define PIN_USB_DP          (18ul)

#ifdef __cplusplus
}
#endif

/*----------------------------------------------------------------------------
 *        Arduino objects - C++ only
 *----------------------------------------------------------------------------*/

#ifdef __cplusplus

/*	=========================
 *	===== SERCOM DEFINITION
 *	=========================
*/
extern SERCOM sercom0;
extern SERCOM sercom1;
extern SERCOM sercom3;

extern Uart Serial1;

#endif

// These serial port names are intended to allow libraries and architecture-neutral
// sketches to automatically default to the correct port name for a particular type
// of use.  For example, a GPS module would normally connect to SERIAL_PORT_HARDWARE_OPEN,
// the first hardware serial port whose RX/TX pins are not dedicated to another use.
//
// SERIAL_PORT_MONITOR        Port which normally prints to the Arduino Serial Monitor
//
// SERIAL_PORT_USBVIRTUAL     Port which is USB virtual serial
//
// SERIAL_PORT_LINUXBRIDGE    Port which connects to a Linux system via Bridge library
//
// SERIAL_PORT_HARDWARE       Hardware serial port, physical RX & TX pins.
//
// SERIAL_PORT_HARDWARE_OPEN  Hardware serial ports which are open for use.  Their RX & TX
//                            pins are NOT connected to anything by default.
#define SERIAL_PORT_USBVIRTUAL      Serial
#define SERIAL_PORT_MONITOR         Serial
// Serial has no physical pins broken out, so it's not listed as HARDWARE port
#define SERIAL_PORT_HARDWARE        Serial1
#define SERIAL_PORT_HARDWARE_OPEN   Serial1

#endif /* _VARIANT_DYNALORA_USB_ */

