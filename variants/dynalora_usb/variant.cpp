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


#include "variant.h"
/*
 * Pins descriptions
 */
const PinDescription g_APinDescription[]=
{
  // 0..9 - Digital pins
  // ----------------------
  { PORTA,  0, PIO_SERCOM_ALT, (PIN_ATTR_DIGITAL|PIN_ATTR_EXTINT), No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_0 }, // D0 - SDA - TX - SDO
  { PORTA, 1, PIO_SERCOM_ALT, (PIN_ATTR_DIGITAL|PIN_ATTR_EXTINT), No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_1 }, // D1 - SCL - RX - SCK
  { PORTA, 2, PIO_SERCOM_ALT, (PIN_ATTR_PWM|PIN_ATTR_DIGITAL|PIN_ATTR_ANALOG|PIN_ATTR_EXTINT), ADC_Channel0, PWM3_CH0, NOT_ON_TIMER, EXTERNAL_INT_2 }, // D2 - SDI
  { PORTA, 30, PIO_SERCOM_ALT, (PIN_ATTR_PWM|PIN_ATTR_DIGITAL|PIN_ATTR_EXTINT), No_ADC_Channel, PWM1_CH0, NOT_ON_TIMER, EXTERNAL_INT_10 }, // D3 - SWCLK
  { PORTA, 31, PIO_SERCOM_ALT, (PIN_ATTR_PWM|PIN_ATTR_DIGITAL|PIN_ATTR_EXTINT), No_ADC_Channel, PWM1_CH1, NOT_ON_TIMER, EXTERNAL_INT_11 }, // D4 - SWDIO
  
  { PORTA,  15, PIO_DIGITAL, (PIN_ATTR_DIGITAL|PIN_ATTR_EXTINT), No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_15 }, // BUTTON
  { PORTA,  3, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, // SD_CS
  { PORTA,  9, PIO_DIGITAL, (PIN_ATTR_DIGITAL|PIN_ATTR_EXTINT), No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_9 }, // RF_INT
  { PORTA,  10, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, // RF_RESET
  { PORTA,  11, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, // RF_CS
    
  // 10..11 (LEDs)
  // ----------------------
  { PORTA,  27, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, // LED
  { PORTA,  19, PIO_DIGITAL, (PIN_ATTR_DIGITAL|PIN_ATTR_PWM|PIN_ATTR_TIMER), No_ADC_Channel, PWM3_CH1, TCC0_CH2, EXTERNAL_INT_NONE }, // Neopixel RGB
  
  // 12 - Alternate use of A0 (DAC output)
  // ----------------------
  { PORTA,  2, PIO_ANALOG, PIN_ATTR_ANALOG, DAC_Channel0, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, // VOUT

  // 13..15 - USB
  // --------------------
  { NOT_A_PORT, 0, PIO_NOT_A_PIN, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, // USB Host enable DOES NOT EXIST ON THIS BOARD
  { PORTA, 24, PIO_COM, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, // USB/DM
  { PORTA, 25, PIO_COM, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, // USB/DP

  // ----------------------
  // 16..19 - SPI Flash
  { PORTA, 5, PIO_SERCOM_ALT, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, // SDI: SERCOM0/PAD[1]
  { PORTA, 4, PIO_SERCOM_ALT, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, // SDO: SERCOM0/PAD[0]
  { PORTA, 7, PIO_SERCOM_ALT, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, // SCK: SERCOM0/PAD[3]
  { PORTA, 6, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_4 }, // CS
    
} ;

const void* g_apTCInstances[TCC_INST_NUM+TC_INST_NUM]={ TCC0, TCC1, TCC2, TC3, TC4, TC5 } ;

// Multi-serial objects instantiation
SERCOM sercom0( SERCOM0 ) ;
SERCOM sercom1( SERCOM1 ) ;

Uart Serial1( &sercom1, PIN_SERIAL1_RX, PIN_SERIAL1_TX, PAD_SERIAL1_RX, PAD_SERIAL1_TX ) ;

void SERCOM1_Handler()
{
  Serial1.IrqHandler();
}

void WIRE_IT_HANDLER(void) __attribute__ ((weak));
