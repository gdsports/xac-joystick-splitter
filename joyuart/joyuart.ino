/***************************************************************************
MIT License

Copyright (c) 2018 gdsports625@gmail.com

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
****************************************************************************/
/*
   Teensy LC USB joystick with 16 or 32 buttons and 6 axis input

   You must select Joystick from the "Tools > USB Type" menu

Add support for Adafruit ItsyBitsy 32U4. Should also work for other 32u4 boards
such as Arduino Micro and SparkFun/Arduino/clone Pro Micro. But the #if must be
updated. 5V is OK because 32u4 outputs are not connected to any 3.3V inputs.

*/

#define ARDUINO_AVR_ITSYBITSY32U4    (defined(ARDUINO_AVR_ITSYBITSY32U4_3V)||defined(ARDUINO_AVR_ITSYBITSY32U4_5V))

#if ARDUINO_AVR_ITSYBITSY32U4
#define SerialDebug     if(0)Serial1

#include "Joystick.h"   // https://github.com/MHeironimus/ArduinoJoystickLibrary/Joystick
Joystick_ Joystick(0x03, JOYSTICK_TYPE_JOYSTICK,
    12,                 // Button count
    0,                  // Hat switches
    true, true, false,  // X, Y, Z axes
    false, false, false,// X, Y, Z rotation axes
    false, false, false,// rudder, throttle, accelerator
    false, false        // brake, steering
    );
// configure the joystick to manual send mode.  This gives precise
// control over when the computer receives updates, but it does
// require you to manually call Joystick.sendState().
#define BEGIN() begin(false)
#define SETXAXIS(x) setXAxis(x)
#define SETYAXIS(x) setYAxis(x)
#define SETBUTTON(button_num, value)    setButton(button_num, value)
#define SENDSTATE() sendState()
#else
// Default is Teensy 3 or LC
#define SerialDebug if(0)Serial2
  // configure the joystick to manual send mode.  This gives precise
  // control over when the computer receives updates, but it does
  // require you to manually call Joystick.send_now().
#define BEGIN() useManualSend(true)
#define SETXAXIS(x) X(x)
#define SETYAXIS(x) Y(x)
#define SETBUTTON(button_num, value)    button(((button_num)+1), (value))
#define SENDSTATE() send_now()
#endif

// Configure the number of buttons.  Be careful not
// to use a pin for both a digital button and analog
// axis.  The pullup resistor will interfere with
// the analog voltage.
const int numButtons = 16;  // 16 for Teensy, 32 for Teensy++

struct GamePadEventData
{
  union { //axes and hut switch
    uint32_t axes;
    struct {
      uint32_t x : 10;
      uint32_t y : 10;
      uint32_t hat : 4;
      uint32_t twist : 8;
    };
  };
  uint8_t buttons_a;
  uint8_t slider;
  uint8_t buttons_b;
} __attribute__((packed));

uint8_t HIDrptLen;

uint8_t HIDrpt[sizeof(GamePadEventData)], HIDrptOld[sizeof(GamePadEventData)];
char HIDrptCstr[64];

void setup() {
  // you can print to the serial monitor while the joystick is active!
  SerialDebug.begin(115200);
  // USB HID report received on this port.
  Serial1.begin(115200);

  Joystick.BEGIN();

  memset(HIDrptOld, 0xFF, sizeof(HIDrptOld));
  HIDrptLen = 0;
  SerialDebug.println("Begin Complete Joystick Test");
}

void analyze_HIDrpt(const struct GamePadEventData *evt)
{
  SerialDebug.print("X: ");
  SerialDebug.print(evt->x, HEX);
  Joystick.SETXAXIS(evt->x);
  SerialDebug.print(" Y: ");
  SerialDebug.print(evt->y, HEX);
  Joystick.SETYAXIS(evt->y);
  SerialDebug.print(" Buttons A: ");
  SerialDebug.print(evt->buttons_a, HEX);
  for (int i = 0; i < 8; i++) {
    Joystick.SETBUTTON(i, (evt->buttons_a & (1<<i))!=0);
  }
  SerialDebug.print(" Buttons B: ");
  SerialDebug.print(evt->buttons_b, HEX);
  for (int i = 0; i < 8; i++) {
    Joystick.SETBUTTON(i+8, (evt->buttons_b & (1<<i))!=0);
  }
  SerialDebug.println();
  Joystick.SENDSTATE();
}

uint8_t deserialize(char *buf, uint8_t *outbuf, uint8_t outbuf_len)
{
  char *p;
  uint8_t outbuf_count = 0;

  memset(outbuf, 0, outbuf_len);
  p = strtok(buf, ",");
  while (p != NULL && outbuf_count < outbuf_len) {
    *outbuf++ = strtoul(p, NULL, 16);
    outbuf_count++;
    p = strtok(NULL, ",");
  }
  return outbuf_count;
}

bool InPacket=false;

void loop() {
  // Read joystick HID report from Serial1
  if (Serial1.available() > 0) {
    int b = Serial1.read();
    if (b != -1) {
      if (b == '\n') {
        HIDrptCstr[HIDrptLen] = '\0';
        HIDrptLen = deserialize(HIDrptCstr, HIDrpt, sizeof(HIDrpt));
        if (HIDrptLen == sizeof(HIDrpt)) {
          // Ignore duplicate reports
          if (memcmp(HIDrpt, HIDrptOld, sizeof(HIDrptOld)) != 0) {
            analyze_HIDrpt((const GamePadEventData*)HIDrpt);
            memcpy(HIDrptOld, HIDrpt, sizeof(HIDrptOld));
          }
        }
        HIDrptLen = 0;
      }
      else {
        HIDrptCstr[HIDrptLen] = (char)b;
        if (HIDrptLen < sizeof(HIDrptCstr)-2) HIDrptLen++;
      }
    }
  }
}
