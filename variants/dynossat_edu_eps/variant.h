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

#ifndef _VARIANT_DYNOSSAT_EDU_EPS_
#define _VARIANT_DYNOSSAT_EDU_EPS_

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
#define PINS_COUNT           (33u)
#define NUM_DIGITAL_PINS     (5u)
#define NUM_ANALOG_INPUTS    (2u)
#define NUM_ANALOG_OUTPUTS   (0u)
#define analogInputToDigitalPin(p)  ((p < 6u) ? (p) + 14u : -1)

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
#define PIN_LED_13           (13u)
#define PIN_LED              PIN_LED_13
#define LED_BUILTIN          PIN_LED_13
#define NEOPIXEL_BUILTIN     PIN_LED_13
#define PIN_NEOPIXEL		 NEOPIXEL_BUILTIN

/*
 * Analog pins
 */
#define PIN_A0               (14ul)
#define PIN_A1               (PIN_A0 + 1)
#define PIN_3V3              PIN_A0
#define PIN_5V				 PIN_A1

static const uint8_t A0  = PIN_A0;
static const uint8_t A1  = PIN_A1;

static const uint8_t V_3V3_MEAS_PIN  = PIN_A0;
static const uint8_t V_5V_MEAS_PIN  = PIN_A1;

#define PIN_DAC0			0

#define ADC_RESOLUTION		12

/*
 * Digital pins
 */
#define PIN_D0               (0ul)
#define PIN_D1               (PIN_D0 + 1)
#define PIN_D2               (PIN_D0 + 2)
#define PIN_D3               (PIN_D0 + 3)
#define PIN_D4               (PIN_D0 + 4)
#define PIN_D26              (PIN_D0 + 26)
#define PIN_D27              (PIN_D0 + 27)

static const uint8_t D0  = PIN_D0;
static const uint8_t D1  = PIN_D1;
static const uint8_t D2  = PIN_D2;
static const uint8_t D3  = PIN_D3;
static const uint8_t D4  = PIN_D4;
static const uint8_t D26 = PIN_D26;
static const uint8_t D27 = PIN_D27;


static const uint8_t OVTEMP = PIN_D0;
static const uint8_t SAT_PWR_EN = PIN_D1;
static const uint8_t SAT_RESET = PIN_D2;
static const uint8_t INT_IMU = PIN_D3;
static const uint8_t PWRMON_ALERT = PIN_D4;

// On-board SPI Flash
#define EXTERNAL_FLASH_DEVICES  GD25Q32C
#define EXTERNAL_FLASH_USE_SPI  SPI1
#define EXTERNAL_FLASH_USE_CS   SS1

/*
 * Serial interfaces
 */

// Serial1 - SERCOM1
#define PIN_SERIAL1_RX       (25ul)
#define PIN_SERIAL1_TX       (24ul)
#define PAD_SERIAL1_TX       (UART_TX_PAD_0)
#define PAD_SERIAL1_RX       (SERCOM_RX_PAD_1)

static const uint8_t TX = PIN_SERIAL1_TX;
static const uint8_t RX = PIN_SERIAL1_RX;

/*
 * SPI Interfaces
 */
#define SPI_INTERFACES_COUNT 2

#define PIN_SPI_MISO         (18u)
#define PIN_SPI_MOSI         (19u)
#define PIN_SPI_SCK          (20u)
#define PERIPH_SPI           sercom4
#define PAD_SPI_TX           SPI_PAD_2_SCK_3
#define PAD_SPI_RX           SERCOM_RX_PAD_0

//static const uint8_t SS	  = PIN_A2 ;	// SERCOM4 last PAD is present on A2 but HW SS isn't used. Set here only for reference.
static const uint8_t MOSI = PIN_SPI_MOSI ;
static const uint8_t MISO = PIN_SPI_MISO ;
static const uint8_t SCK  = PIN_SPI_SCK ;

static const uint8_t SDO = PIN_SPI_MOSI ;
static const uint8_t SDI = PIN_SPI_MISO ;

#define PIN_SPI1_MISO         (28u)
#define PIN_SPI1_MOSI         (29u)
#define PIN_SPI1_SCK          (30u)
#define PERIPH_SPI1           sercom3
#define PAD_SPI1_TX           SPI_PAD_0_SCK_1
#define PAD_SPI1_RX           SERCOM_RX_PAD_3

static const uint8_t SS1   = 31 ;
static const uint8_t MOSI1 = PIN_SPI1_MOSI ;
static const uint8_t MISO1 = PIN_SPI1_MISO ;
static const uint8_t SCK1  = PIN_SPI1_SCK ;

static const uint8_t SDO1 = PIN_SPI1_MOSI ;
static const uint8_t SDI1 = PIN_SPI1_MISO ;

static const uint8_t FLASH_SDO = PIN_SPI1_MOSI ;
static const uint8_t FLASH_SDI = PIN_SPI1_MISO ;
static const uint8_t FLASH_SCK = PIN_SPI1_SCK ;
static const uint8_t FLASH_CS = SS1 ;


/*
 * Wire Interfaces
 */
#define WIRE_INTERFACES_COUNT 2

#define PIN_WIRE_SDA         (16u)
#define PIN_WIRE_SCL         (17u)
#define PERIPH_WIRE          sercom2
#define WIRE_IT_HANDLER      SERCOM2_Handler

static const uint8_t SDA = PIN_WIRE_SDA;
static const uint8_t SCL = PIN_WIRE_SCL;

#define PIN_WIRE1_SDA         (32u)
#define PIN_WIRE1_SCL         (33u)
#define PERIPH_WIRE1          sercom0
#define WIRE1_IT_HANDLER      SERCOM0_Handler

static const uint8_t SDA1 = PIN_WIRE1_SDA;
static const uint8_t SCL1 = PIN_WIRE1_SCL;

static const uint8_t PWRMON_SDA = PIN_WIRE1_SDA;
static const uint8_t PWRMON_SCL = PIN_WIRE1_SCL;

/*
 * USB
 */
#define PIN_USB_HOST_ENABLE (21ul)
#define PIN_USB_DM          (22ul)
#define PIN_USB_DP          (23ul)

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
extern SERCOM sercom2;
extern SERCOM sercom3;
extern SERCOM sercom4;
extern SERCOM sercom5;

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

#endif /* _VARIANT_DYNOSSAT_EDU_EPS_ */

