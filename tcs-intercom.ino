/*
 * TCS Intercom Controller
 *
 * Controls the TCS home intercom system by implementing its proprietary bus protocol.
 *
 * This sketch has been implemented and tested on an ATMega328P based Arduino Pro Mini
 * compatible board.
 *
 * In order to eliminate the bootloader delay, it necessary to flash the modified
 * Arduino bootloarder by following the instructions in:
 * https://github.com/microfarad-de/bootloader
 *
 * This source file is part of the follwoing repository:
 * http://www.github.com/microfarad-de/tcs-intercom
 *
 * Please visit:
 *   http://www.microfarad.de
 *   http://www.github.com/microfarad-de
 *
 * Copyright (C) 2023 Karim Hraibi (khraibi@gmail.com)
 *
 * The following license agreement applies unless stated otherwise within
 * the code comments.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Version: 1.0.0
 * Date:    May 09, 2023
 */
#define VERSION_MAJOR 1  // Major version
#define VERSION_MINOR 0  // Minor version
#define VERSION_MAINT 0  // Maintenance version

#include <Arduino.h>
#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/wdt.h>
#include "src/Cli/Cli.h"
#include "src/Nvm/Nvm.h"



/*
 * Pin assignment
 */
#define OUTPUT_PIN 5

/*
 * Configuration parameters
 */
#define SERIAL_BAUD      115200  // Serial communication baud rate
#define START_BIT        6       // ms
#define ONE_BIT          4       // ms
#define ZERO_BIT         2       // ms
#define SERIAL_NO        240011  // 3A98B Imdoor unit serial number
#define CMD_OPEN_DOOR    0x13A98B80
#define CMD_OUTDOOR_RING 0x03A98B81 // 0x03A98B80
#define CMD_INDOOR_RING  0x13A98B41
#define CMD_

/*
 * Global variables
 */
volatile uint32_t CMD = 0;
volatile uint8_t lengthCMD = 0;
volatile bool cmdReady;

/*
 * Function declarations
 */
void sendeProtokollHEX(uint32_t protokoll);
int  cmdOpenDoor (int argc, char **argv);
int  cmdOutdoorRing (int argc, char **argv);
int  cmdIndoorRing (int argc, char **argv);
int  cmdTest (int argc, char **argv);
void powerSave (void);




void setup() {
  pinMode(OUTPUT_PIN, OUTPUT);
  digitalWrite(OUTPUT_PIN, LOW);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  ADCSRA &= ~_BV(ADEN);             // Disable ADC, see ATmega328P datasheet Section 28.9.2
  power_adc_disable ();             //

  ACSR =
    (0 << ACD) |    // Analog Comparator: Enabled
    (0 << ACBG) |   // Analog Comparator Bandgap Select: AIN0 is applied to the positive input
    (0 << ACO) |    // Analog Comparator Output: Off
    (1 << ACI) |    // Analog Comparator Interrupt Flag: Clear Pending Interrupt
    (1 << ACIE) |   // Analog Comparator Interrupt: Enabled
    (0 << ACIC) |   // Analog Comparator Input Capture: Disabled
    (0 << ACIS1) | (0 << ACIS0);   // Analog Comparator Interrupt Mode: Comparator Interrupt on changing Edge

  Cli.init ( SERIAL_BAUD );

  Serial.println ("");
  Serial.println (F("+ + +  T C S  I N T E R C O M  C O N T R O L  + + +"));
  Serial.println ("");
  Cli.xprintf    ("V %d.%d.%d\r\n", VERSION_MAJOR, VERSION_MINOR, VERSION_MAINT);
  Serial.println ("");
  Cli.newCmd     ("o" , "Open door" , cmdOpenDoor);
  Cli.newCmd     ("r" , "Outdoor ring" , cmdOutdoorRing);
  Cli.newCmd     ("i" , "Indoor ring" , cmdIndoorRing);
  //Cli.newCmd     ("t" , "Test routine" , cmdTest);
  Cli.showHelp ();
}

String inData;
void loop() {
  static uint32_t ledTs = 0;
  uint32_t ts = millis ();

  Cli.getCmd ();

  if (cmdReady) {
    cmdReady = 0;
    if (lengthCMD == 1) {
      Cli.xprintf("Received command: 0x%08lX\r\n", CMD);
    }
    else {
      Cli.xprintf("Received command: 0x%04lX\r\n", CMD);
    }

    if (((CMD ^ CMD_OUTDOOR_RING) & 0xFFFFFF00) == 0) {
      delay(1000);
      Serial.println (F("Opening door"));
      sendeProtokollHEX (CMD_OPEN_DOOR);
    }

    digitalWrite (LED_BUILTIN, HIGH);
    ledTs = ts;
  }

  if (ts - ledTs >= 1000) {
    digitalWrite (LED_BUILTIN, LOW);
  }

  //powerSave ();
}


int cmdOpenDoor (int argc, char **argv) {
  Cli.xprintf ("Sent command: 0x%08lX\r\n", CMD_OPEN_DOOR);
  sendeProtokollHEX (CMD_OPEN_DOOR);
  return 0;
}

int cmdOutdoorRing (int argc, char **argv) {
  Cli.xprintf ("Sent command: 0x%08lX\r\n", CMD_OUTDOOR_RING);
  sendeProtokollHEX (CMD_OUTDOOR_RING);
  return 0;
}

int cmdIndoorRing (int argc, char **argv) {
  Cli.xprintf ("Sent command: 0x%08lX\r\n", CMD_INDOOR_RING);
  sendeProtokollHEX (CMD_INDOOR_RING);
  return 0;
}


int cmdTest (int argc, char **argv) {
  Serial.println(F("Running test routine"));
  digitalWrite(OUTPUT_PIN, HIGH);
  delay(2000);
  digitalWrite(OUTPUT_PIN, LOW);
  return 0;
}


void powerSave (void) {
  // enter sleep, wakeup will be triggered by the
  // next Timer 0 interrupt (millis tick)
  set_sleep_mode (SLEEP_MODE_IDLE); // configure lowest sleep mode that keeps clk_IO for Timer 1
  cli ();
  sleep_enable ();
  sei ();
  sleep_cpu ();
  sleep_disable ();
}


//it is better to also give an arg with the length because it
//is possible that there is a 4Byte protokoll smaller than 0xFFFF
//something like void sendeProtokollHEX(uint32_t protokoll,byte firstBit) {
//  int length = 16;
//  byte checksm = 1;
//  if (firstBit) length = 32;
//and so on...
void sendeProtokollHEX(uint32_t protokoll) {
  int length = 16;
  byte checksm = 1;
  byte firstBit = 0;
  if (protokoll > 0xFFFF) {
    length = 32;
    firstBit = 1;
  }
  digitalWrite(OUTPUT_PIN, HIGH);
  delay(START_BIT);
  digitalWrite(OUTPUT_PIN, !digitalRead(OUTPUT_PIN));
  delay(firstBit ? ONE_BIT : ZERO_BIT);
  int curBit = 0;
  for (byte i = length; i > 0; i--) {
    curBit = bitRead(protokoll, i - 1);
    digitalWrite(OUTPUT_PIN, !digitalRead(OUTPUT_PIN));
    delay(curBit ? ONE_BIT : ZERO_BIT);
    checksm ^= curBit;
  }
  digitalWrite(OUTPUT_PIN, !digitalRead(OUTPUT_PIN));
  delay(checksm ? ONE_BIT : ZERO_BIT);
  digitalWrite(OUTPUT_PIN, LOW);
}


ISR(ANALOG_COMP_vect ) {
  static uint32_t curCMD;
  static uint32_t usLast;
  static byte curCRC;
  static byte calCRC;
  static byte curLength;
  static byte cmdIntReady;
  static byte curPos;
  uint32_t usNow = micros();
  uint32_t timeInUS = usNow - usLast;
  usLast = usNow;
  byte curBit = 4;
  if (timeInUS < 1000) {
    curBit = 5; // invalid glitches typical 29ms
  } else if (timeInUS >= 1000 && timeInUS <= 2999) {
    curBit = 0;
  } else if (timeInUS >= 3000 && timeInUS <= 4999) {
    curBit = 1;
  } else if (timeInUS >= 5000 && timeInUS <= 6999) {
    curBit = 2;
  } else if (timeInUS >= 7000 && timeInUS <= 24000) {
    curBit = 3;
    curPos = 0;
  }

  if (curBit != 5) { // skip processing for glitches
    if (curPos == 0) {
      if (curBit == 2) {
        curPos++;
      }
      curCMD = 0;
      curCRC = 0;
      calCRC = 1;
      curLength = 0;
    } else if (curBit == 0 || curBit == 1) {
      if (curPos == 1) {
        curLength = curBit;
        curPos++;
      } else if (curPos >= 2 && curPos <= 17) {
        if (curBit)bitSet(curCMD, (curLength ? 33 : 17) - curPos);
        calCRC ^= curBit;
        curPos++;
      } else if (curPos == 18) {
        if (curLength) {
          if (curBit)bitSet(curCMD, 33 - curPos);
          calCRC ^= curBit;
          curPos++;
        } else {
          curCRC = curBit;
          cmdIntReady = 1;
        }
      } else if (curPos >= 19 && curPos <= 33) {
        if (curBit)bitSet(curCMD, 33 - curPos);
        calCRC ^= curBit;
        curPos++;
      } else if (curPos == 34) {
        curCRC = curBit;
        cmdIntReady = 1;
      }
    } else {
      curPos = 0;
    }
    if (cmdIntReady) {
      cmdIntReady = 0;
      curPos = 0;
      if (curCRC == calCRC) {
        cmdReady = 1;
        lengthCMD = curLength;
        CMD = curCMD;
      }
      curCMD = 0;
    }
  }
}
