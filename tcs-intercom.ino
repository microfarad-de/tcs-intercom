/*
 * TCS Intercom Controller
 *
 * Controls the TCS home intercom system by implementing its proprietary bus protocol.
 *
 * This sketch has been implemented and tested on an ATMega328P based Arduino Pro Mini
 * compatible board.
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
 * Version: 1.2.0
 * Date:    May 29, 2023
 */
#define VERSION_MAJOR 1  // Major version
#define VERSION_MINOR 2  // Minor version
#define VERSION_MAINT 0  // Maintenance version

#include <Arduino.h>
#include <avr/sleep.h>
#include <avr/power.h>
#include "src/Cli/Cli.h"
#include "src/Nvm/Nvm.h"
#include "src/Led/Led.h"

// Enable debug code
#define DEBUG(_X) _X

/*
 * Pin assignment
 */
#define OUTPUT_PIN 5

/*
 * Configuration parameters
 */
#define SERIAL_BAUD          38400   // Serial communication baud rate
#define START_BIT            6       // Protocol start bit duration in ms
#define ONE_BIT              4       // Protocol 1 bit duration in ms
#define ZERO_BIT             2       // Protocol 0 bit duration in ms
#define ENTRY_CODE_SIZE      8       // Entry code array size
#define ENTRY_CODE_THR       1000    // Entry code duration threshold in ms
#define ENTRY_CODE_TIMEOUT   5000    // Entry code timeout duration in ms
#define PSV_DISABLE_DURATION 500     // Power save disabling duration in ms
#define OPEN_DOOR_DELAY      500     // Delay in ms before opening the door
#define DEFAULT_SERIAL_NO    12345   // Default indoor unit serial number

/*
 * Known intercom commands
 */
#define CMD_OPEN_DOOR    0x10000080
#define CMD_OUTDOOR_RING 0x00000080
#define CMD_INDOOR_RING  0x10000041
#define CMD_LIGHT_ON     0x1200
#define CMD_CUSTOM_BTN   0x60000008
#define CMD_SERIAL_MASK  0x0FFFFF00  // Mask the serial number bits of a command
#define CMD_CONSTRUCT(_CMD) (_CMD & (~CMD_SERIAL_MASK)) | ((Nvm.serialNo << 8) & CMD_SERIAL_MASK);  // Constructs a command by adding the serial No


// Command Length
enum CmdLength_e {
  LEN_16BIT  = 0,
  LEN_32BIT  = 1
};


// Entry code values
enum EntryCode_e {
  CODE_SHORT = 0,   // Corresponds to a short delay between two consecutive ring button presses
  CODE_LONG  = 1,   // Corresponds to a long delay between two consecutive ring button presses
  CODE_END   = 255  // Corresponds to the end of the code sequence
};


// Volatile variables used inside the ISR
struct {
  volatile uint32_t    cmd     = 0;            // Decoded command payload
  volatile CmdLength_e cmdLength = LEN_16BIT;  // Command length
  volatile bool        cmdReady;               // Command ready flag
} Isr;


// Configuration parameters stored in EEPROM (Nvm)
struct Nvm_t {
  uint32_t serialNo;                   // Indoor unit serial number
  uint8_t  entryCode[ENTRY_CODE_SIZE]; // Entry code characters of type EntryCode_e
} Nvm;


// NVM backup copy
Nvm_t NvmBak;


// LED object
LedClass Led;


/*
 * Function declarations
 */
bool nvmValidate       (void);
void nvmRead           (void);
void nvmWrite          (void);
void sendTcsBusCommand (uint32_t cmd, CmdLength_e cmdLen = LEN_32BIT);
void sendCommand       (uint32_t cmd, CmdLength_e cmdLen = LEN_32BIT);
void powerSave         (void);
int  cmdOpenDoor       (int argc, char **argv);
int  cmdOutdoorRing    (int argc, char **argv);
int  cmdIndoorRing     (int argc, char **argv);
int  cmdTest           (int argc, char **argv);
int  cmdRom            (int argc, char **argv);
int  cmdSetSerial      (int argc, char **argv);
int  cmdSetEntryCode   (int argc, char **argv);


/*
 * Arduino initialization routine
 */
void setup() {
  pinMode      (OUTPUT_PIN, OUTPUT);
  digitalWrite (OUTPUT_PIN, LOW);

  // Disable non needed peripherals for power saving
  ADCSRA &= ~_BV(ADEN);    // Disable ADC, see ATmega328P datasheet Section 28.9.2
  power_all_disable   ();  // Disable peripherals
  power_usart0_enable ();  // Enable serial port
  power_timer0_enable ();  // Enable Timer 0 (required for millis() and micros())

  ACSR =
    (0 << ACD)   |  // Analog comparator: Enabled
    (0 << ACBG)  |  // Analog comparator bandgap Select: AIN0 is applied to the positive input
    (0 << ACO)   |  // Analog comparator output: Off
    (1 << ACI)   |  // Analog comparator interrupt Flag: Clear pending interrupt
    (1 << ACIE)  |  // Analog comparator interrupt: Enabled
    (0 << ACIC)  |  // Analog comparator input capture: Disabled
    (0 << ACIS1) | (0 << ACIS0);  // Analog comparator interrupt mode: Comparator Interrupt on changing edge

  Led.initialize (LED_BUILTIN);
  Cli.init ( SERIAL_BAUD );

  Serial.println (F("\r\n+ + +  I N T E R C O M  C O N T R O L  + + +\r\n"));
  Cli.xprintf    ("V %d.%d.%d\r\n\r\n", VERSION_MAJOR, VERSION_MINOR, VERSION_MAINT);
  Cli.newCmd     ("o"      , "Open door"    , cmdOpenDoor);
  Cli.newCmd     ("or"     , "Outdoor ring" , cmdOutdoorRing);
  Cli.newCmd     ("ir"     , "Indoor ring"  , cmdIndoorRing);
  Cli.newCmd     ("config" , "Show the system configuration", cmdRom);
  Cli.newCmd     ("r"      , "Show the system configuration", cmdRom);
  Cli.newCmd     ("code"   , "Set the door entry code (arg: [binary code, eg. 1101])", cmdSetEntryCode);
  Cli.newCmd     ("serial" , "Set the indoor unit serial number (arg: <number>)"     , cmdSetSerial);
  Cli.showHelp   ();

  nvmRead ();
}


/*
 * Arduino main loop
 */
void loop () {
  static enum {STATE_WAIT, STATE_READ, STATE_OPEN} state = STATE_WAIT;
  static uint32_t cmdTs   = 0;
  static uint32_t cmdDt   = 0;
  static uint8_t  codeIdx = 0;
  uint32_t ts          = millis ();
  uint32_t cmdOutRing  = CMD_CONSTRUCT(CMD_OUTDOOR_RING);
  uint32_t cmdInRing   = CMD_CONSTRUCT(CMD_INDOOR_RING);
  bool receivedRingCmd = false;

  Cli.getCmd ();

  Led.loopHandler ();

  if (Isr.cmdReady) {
    Isr.cmdReady = false;

    Led.blink (1, 100, 200);

    Serial.print(F("Received command: "));
    if (Isr.cmdLength == LEN_32BIT) {
      Cli.xprintf ("0x%08lX\r\n\r\n", Isr.cmd);
    }
    else {
      Cli.xprintf ("0x%04lX\r\n\r\n", Isr.cmd);
    }

    // Ring button was pressed
    if (((Isr.cmd ^ cmdOutRing) & 0xFFFFFFF0) == 0 ||
        ((Isr.cmd ^ cmdInRing)  & 0xFFFFFFF0) == 0) {
      Led.blinkStop ();
      cmdDt = ts - cmdTs;
      cmdTs = ts;
      receivedRingCmd = true;
    }
  }

  // Entry code detection state machine
  switch (state) {
    // Waiting for input
    case STATE_WAIT:
      // Command received
      if (receivedRingCmd) {
        receivedRingCmd = false;
        DEBUG(Serial.println("START\r\n");)
        Led.blink (1, 100, 200);
        codeIdx = 0;
        state   = STATE_READ;
      }
      break;

    // Reading user input
    case STATE_READ:
      // Command received
      if (receivedRingCmd) {
        receivedRingCmd = false;
        // Short duration
        if (Nvm.entryCode[codeIdx] == CODE_SHORT) {
          if (cmdDt <= ENTRY_CODE_THR) {
            DEBUG(Cli.xprintf ("SHORT OK idx = %d\r\n\r\n", codeIdx);)
            Led.blink (1, 100, 200);
            codeIdx++;
          }
          else {
            DEBUG(Cli.xprintf ("SHORT NOK idx = %d\r\n\r\n", codeIdx);)
            Led.blink (2, 100, 200);
            state = STATE_WAIT;
          }
        }
        // Long duration
        else if (Nvm.entryCode[codeIdx] == CODE_LONG) {
          if (cmdDt > ENTRY_CODE_THR) {
            DEBUG(Cli.xprintf ("LONG OK idx = %d\r\n\r\n", codeIdx);)
            Led.blink (1, 100, 200);
            codeIdx++;
          }
          else {
            DEBUG(Cli.xprintf ("LONG NOK idx = %d\r\n\r\n", codeIdx);)
            Led.blink (2, 100, 200);
            state = STATE_WAIT;
          }
        }
      }
      // Cmmand end
      if (Nvm.entryCode[codeIdx] == CODE_END) {
        state = STATE_OPEN;
      }
      break;

    // Open the door
    case STATE_OPEN:
      Serial.println("DOOR OPEN\r\n");
      delay(OPEN_DOOR_DELAY);
      Led.blink (1, 1000, 0);
      sendCommand (CMD_OPEN_DOOR, LEN_32BIT);
      state = STATE_WAIT;
      break;

    default:
      break;
  }

  // Code word overflow
  if (codeIdx >= ENTRY_CODE_SIZE) {
    codeIdx = 0;
    state   = STATE_WAIT;
  }

  // Timeout
  if (ts - cmdTs > ENTRY_CODE_TIMEOUT && state != STATE_WAIT) {
    DEBUG(Serial.println("TIMEOUT\r\n");)
    Led.blink (3, 100, 200);
    state = STATE_WAIT;
  }

  if (state == STATE_WAIT && Led.blinking == false) {
    // Sleep
    powerSave ();
  }
}

/*
 * Wrapper funciton for sending TCS bus commands
 */
void sendCommand (uint32_t cmd, CmdLength_e cmdLen) {
  uint32_t c = CMD_CONSTRUCT (cmd);
  Serial.print(F("Sent command: "));
  if (cmdLen == LEN_32BIT) {
    Cli.xprintf ("0x%08lX\r\n\r\n", c);
  }
  else {
    Cli.xprintf ("0x%04lX\r\n\r\n", c);
  }
  sendTcsBusCommand (c, cmdLen);
}


/*
 * Handle open door command
 */
int cmdOpenDoor (int argc, char **argv) {
  sendCommand (CMD_OPEN_DOOR, LEN_32BIT);
  return 0;
}
int cmdOutdoorRing (int argc, char **argv) {
  sendCommand (CMD_OUTDOOR_RING, LEN_32BIT);
  return 0;
}
int cmdIndoorRing (int argc, char **argv) {
  sendCommand (CMD_INDOOR_RING, LEN_32BIT);
  return 0;
}


/*
 * CLI command for setting the device serial number
 */
int cmdSetSerial (int argc, char **argv) {
  if (argc != 2) {
    Cli.xprintf ("Usage: %s <number>\r\n\r\n", argv[0]);
    return 1;
  }
  Nvm.serialNo = atol (argv[1]);
  nvmWrite ();
  cmdRom (1, nullptr);
  return 0;
}


/*
 * CLI command for setting the entry code
 */
int cmdSetEntryCode (int argc, char **argv) {
  uint8_t i;
  if (argc < 2) {
    Nvm.entryCode[0] = CODE_END;
  }
  else {
    for (i = 0; i < ENTRY_CODE_SIZE - 1; i++) {
      char c = argv[1][i];
      if (c == '\0') {
        break;
      }
      else if (c < '0' || c > '1') {
        Serial.println (F("Invalid argument (must be a string of 0/1)\r\n"));
        return 1;
      }
      else {
        Nvm.entryCode[i] = c - '0';
      }
    }
    Nvm.entryCode[i] = CODE_END;
  }
  nvmWrite ();
  cmdRom (1, nullptr);
  return 0;
}


/*
 * CLI command for displaying the EEPROM data
 */
int cmdRom (int argc, char **argv) {
  bool noEntryCode = true;
  Serial.println (F("System configuration:"));
  Cli.xprintf    (  "  Serial No  = %ld (0x%lX)\r\n", Nvm.serialNo, Nvm.serialNo);
  Serial.print   (F("  Entry code = "));
  for (uint8_t i = 0; i < ENTRY_CODE_SIZE; i ++) {
    if (Nvm.entryCode[i] <= CODE_LONG) {
      Cli.xputchar (Nvm.entryCode[i] + '0');
      noEntryCode = false;
    }
    else {
      break;
    }
  }
  if (noEntryCode) {
    Serial.print(F("n/a"));
  }
  Cli.xprintf ("\r\n  V %d.%d.%d\r\n\r\n", VERSION_MAJOR, VERSION_MINOR, VERSION_MAINT);
  return 0;
}


/*
 * Validate EEPROM data
 */
bool nvmValidate (void) {
  bool valid = true;

  for (uint8_t i = 0; i < ENTRY_CODE_SIZE; i++) {
    if (Nvm.entryCode[i] ==  CODE_END) {
      break;
    }
    else if (Nvm.entryCode[i] > CODE_LONG) {
      Nvm.entryCode[i] = CODE_END;
      valid = false;
      break;
    }
    if (i == ENTRY_CODE_SIZE - 1 && Nvm.entryCode[i] != CODE_END) {
      Nvm.entryCode[i] = CODE_END;
      valid = false;
    }
  }

  if ((Nvm.serialNo & 0xFFF00000) != 0 || Nvm.serialNo == 0) {
    if ((NvmBak.serialNo & 0xFFF00000) != 0 || NvmBak.serialNo == 0) {
      Nvm.serialNo = DEFAULT_SERIAL_NO;
    }
    else {
      Nvm.serialNo = NvmBak.serialNo;
    }
    valid = false;
  }

  NvmBak = Nvm;

  if (!valid) {
    Serial.println (F("Validation error"));
    nvmWrite ();
  }
  return valid;
}


/*
 * Read EEPROM data
 */
void nvmRead (void) {
  eepromRead (0x0, (uint8_t*)&Nvm,    sizeof (Nvm_t));
  eepromRead (0x0, (uint8_t*)&NvmBak, sizeof (Nvm_t));
  nvmValidate ();
}


/*
 * Write EEPROM data
 */
void nvmWrite (void) {
  nvmValidate ();
  eepromWrite (0x0, (uint8_t*)&Nvm, sizeof (Nvm));
}


/*
 * Power saving routine
 * Enables CPU sleep mode
 */
void powerSave (void) {
  static uint32_t enTs = 0;
  uint32_t        ts   = millis ();

  // Delay power saving
  if (ts - enTs <= PSV_DISABLE_DURATION) {
    return;
  }

  power_timer0_disable ();           // Disables millis() timer0 tick
  set_sleep_mode (SLEEP_MODE_IDLE);  // Lowest power mode supports wakeup on analog compare
  cli ();
  sleep_enable ();                   // Enter sleep, wakeup will be triggered by the next analog compare interrupt
  sei ();
  sleep_cpu ();
  sleep_disable ();
  power_timer0_enable ();

  // Disable power save for PSV_DISABLE_DUR
  ts   = millis ();
  enTs = ts;
}


/*
 * Send a message to the TCS bus
 * Credit: https://github.com/atc1441/TCSintercomArduino
 *
 * cmd:     Command (16 or 32 bit long)
 * cmdLen:  Command length
 */
void sendTcsBusCommand (uint32_t cmd, CmdLength_e cmdLen) {
  uint8_t length = 16;
  uint8_t checksm = 1;
  uint8_t firstBit = 0;
  if (cmdLen == LEN_32BIT) {
    length = 32;
    firstBit = 1;
  }
  digitalWrite(OUTPUT_PIN, HIGH);
  delay(START_BIT);
  digitalWrite(OUTPUT_PIN, !digitalRead(OUTPUT_PIN));
  delay(firstBit ? ONE_BIT : ZERO_BIT);
  uint8_t curBit = 0;
  for (uint8_t i = length; i > 0; i--) {
    curBit = bitRead(cmd, i - 1);
    digitalWrite(OUTPUT_PIN, !digitalRead(OUTPUT_PIN));
    delay(curBit ? ONE_BIT : ZERO_BIT);
    checksm ^= curBit;
  }
  digitalWrite(OUTPUT_PIN, !digitalRead(OUTPUT_PIN));
  delay(checksm ? ONE_BIT : ZERO_BIT);
  digitalWrite(OUTPUT_PIN, LOW);
}


/*
 * Analog comparator ISR for detecting valid TCS bus commands
 * Credit: https://github.com/atc1441/TCSintercomArduino
 */
ISR (ANALOG_COMP_vect ) {
  static uint32_t curCmd;
  static uint32_t usLast;
  static uint8_t curCRC;
  static uint8_t calCRC;
  static uint8_t curLength;
  static uint8_t cmdIntReady;
  static uint8_t curPos;
  uint32_t usNow = micros();
  uint32_t timeInUS = usNow - usLast;
  usLast = usNow;
  uint8_t curBit = 4;

  if (timeInUS < 1000) {
    curBit = 5;  // Invalid glitches typical 29ms
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

  if (curBit != 5) {  // Skip processing for glitches
    if (curPos == 0) {
      if (curBit == 2) {
        curPos++;
      }
      curCmd = 0;
      curCRC = 0;
      calCRC = 1;
      curLength = 0;
    } else if (curBit == 0 || curBit == 1) {
      if (curPos == 1) {
        curLength = curBit;
        curPos++;
      } else if (curPos >= 2 && curPos <= 17) {
        if (curBit) bitSet(curCmd, (curLength ? 33 : 17) - curPos);
        calCRC ^= curBit;
        curPos++;
      } else if (curPos == 18) {
        if (curLength) {
          if (curBit) bitSet(curCmd, 33 - curPos);
          calCRC ^= curBit;
          curPos++;
        } else {
          curCRC = curBit;
          cmdIntReady = 1;
        }
      } else if (curPos >= 19 && curPos <= 33) {
        if (curBit) bitSet(curCmd, 33 - curPos);
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
        Isr.cmdReady = true;
        Isr.cmdLength = curLength ? LEN_32BIT : LEN_16BIT;
        Isr.cmd = curCmd;
      }
      curCmd = 0;
    }
  }
}
