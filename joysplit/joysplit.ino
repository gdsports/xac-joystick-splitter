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
 * The Microsoft Xbox Adaptive Controller (XAC) ignores many features of the
 * Logitech Extreme 3D Pro (le3dp) USB joystick. The XAC ignores the hat
 * switch, twist, throttle, and some of the buttons.
 *
 * The proposed solution is to disguise the le3dp as two USB joysticks.  The
 * le3dp joystick is connected to the Arduino Zero native USB port with a USB
 * OTG to host adaptor. The sketch captures the joystick HID report then
 * generates two HID reports, one for each Teensy LC. Each Teensy LC is
 * emulating a USB joystick.
 *
 * Possible substitutions: Replace Arduino Zero with Adafruit ItsyBitsy M0.
 * Replace Teensy LC with Adafruit ItsyBitsy 32u4 3.3V.
 *
 */

// le3dp - Zero - Serial1 - Teensy LC -XAC left  USB joystick port
//              - Serial2 - Teensy LC -XAC right USB joystick port

// le3dp - ItsyBitsy - Serial1 - Teensy LC -XAC left  USB joystick port
//           M0      - Serial2 - Teensy LC -XAC right USB joystick port

/* Left             Right
 * USB              USB
 * 32u4     SAMD    32u4
 * ====     ====    ====
 * GND      GND     GND
 * USB      USB
 * RX-0     TX-1
 *          TX-2    RX-0
 */
/*
 *            Logitech
 *    Left    Extreme   Right
 *    Joy     3D Pro    Joy
 *    ======  ========  =====
 *    X, Y    X, Y
 *            Twist
 *            Throttle
 *            8 way hat X, Y
 */

/*
 * button_a
 * 01   Front trigger
 * 02   Side trigger
 * 04   top lower left
 * 08   top lower right
 * 01   top upper left
 * 02   top upper right
 * 04   base left lower
 * 08   base left upper
 * button_b
 * 01   base center lower
 * 02   base center upper
 * 04   base right lower
 * 08   base right upper
 */

/*
 * XAC Left joystick Buttons
 * 1 X1 = button_b.base_right_lower
 * 2 X2 = button_b.base_right_upper
 * 3 Left stick = button_b.base_center_lower
 * 4 Left bumper = button_a.side.trigger
 * 5 A = button_a.top_lower_left
 * 6 B = button_a.top_lower_right
 * 7 View = button_a.base_left_lower
 * 8 Menu = button_a.base_left_upper
 *
 * XAC Right joystick Buttons
 * 1 View
 * 2 Menu
 * 3 Right stick = button_b.base_center_upper
 * 4 Right bumper = button_a.front_trigger
 * 5 X = button_a.top_upper_left
 * 6 Y = button_a.top_upper_right
 * 7 X1
 * 8 X2
 *
*/
#include <hid.h>
#include <hiduniversal.h>
#include <usbhub.h>

#if defined(ADAFRUIT_ITSYBITSY_M0)
#include <Arduino.h>   // required before wiring_private.h
#include "wiring_private.h" // pinPeripheral() function
#endif

// Serial2 pin and pad definitions (in Arduino files Variant.h & Variant.cpp)
#define PIN_SERIAL2_RX       (34ul)               // Pin description number for PIO_SERCOM on D12
#define PIN_SERIAL2_TX       (36ul)               // Pin description number for PIO_SERCOM on D10
#define PAD_SERIAL2_TX       (UART_TX_PAD_2)      // SERCOM pad 2
#define PAD_SERIAL2_RX       (SERCOM_RX_PAD_3)    // SERCOM pad 3

// Instantiate the Serial2 class
Uart Serial2(&sercom1, PIN_SERIAL2_RX, PIN_SERIAL2_TX, PAD_SERIAL2_RX, PAD_SERIAL2_TX);

void SERCOM1_Handler()    // Interrupt handler for SERCOM1
{
  Serial2.IrqHandler();
}

// On SAMD boards where the native USB port is also the serial console, use
// Serial1 for the serial console. This applies to all SAMD boards except for
// Arduino Zero and M0 boards.
#if (USB_VID==0x2341 && defined(ARDUINO_SAMD_ZERO)) || (USB_VID==0x2a03 && defined(ARDUINO_SAM_ZERO))
#define SerialDebug SERIAL_PORT_MONITOR
#else
#define SerialDebug SERIAL_PORT_MONITOR
#endif

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
  union {
    uint8_t buttons_a;
    struct {
      uint8_t front_trigger : 1;
      uint8_t side_trigger : 1;
      uint8_t top_lower_left : 1;
      uint8_t top_lower_right : 1;
      uint8_t top_upper_left : 1;
      uint8_t top_upper_right : 1;
      uint8_t base_left_lower : 1;
      uint8_t base_left_upper : 1;
    };
  };
  uint8_t slider;
  union {
    uint8_t buttons_b;
    struct {
      uint8_t base_center_lower : 1;
      uint8_t base_center_upper : 1;
      uint8_t base_right_lower : 1;
      uint8_t base_right_upper : 1;
    };
   };
} __attribute__((packed));

struct XACGamePadLeftEventData
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
  union {
    uint8_t buttons_a;
    struct {
      uint8_t X1 : 1;
      uint8_t X2 : 1;
      uint8_t left_stick : 1;
      uint8_t left_bumper : 1;
      uint8_t A : 1;
      uint8_t B : 1;
      uint8_t View : 1;
      uint8_t Menu : 1;
    };
  };
  uint8_t slider;
  uint8_t buttons_b;
} __attribute__((packed));

struct XACGamePadRightEventData
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
  union {
    uint8_t buttons_a;
    struct {
      uint8_t View : 1;
      uint8_t Menu : 1;
      uint8_t right_stick : 1;
      uint8_t right_bumper : 1;
      uint8_t X : 1;
      uint8_t Y : 1;
      uint8_t X1 : 1;
      uint8_t X2 : 1;
    };
  };
  uint8_t slider;
  uint8_t buttons_b;
} __attribute__((packed));

class JoystickEvents
{
  public:
    virtual void OnGamePadChanged(const GamePadEventData *evt);
};

#define RPT_GAMEPAD_LEN	sizeof(GamePadEventData)/sizeof(uint8_t)

class JoystickReportParser : public HIDReportParser
{
  JoystickEvents		*joyEvents;

  uint8_t oldPad[RPT_GAMEPAD_LEN];

  public:
  JoystickReportParser(JoystickEvents *evt);

  virtual void Parse(HID *hid, uint32_t is_rpt_id, uint32_t len, uint8_t *buf);
};

JoystickReportParser::JoystickReportParser(JoystickEvents *evt) :
	joyEvents(evt)
{}

void serialize1(const uint8_t*buf, size_t len) {
  char hidReport[64];
  int hidReportLen;
  hidReportLen = snprintf(hidReport, sizeof(hidReport),
      "%x,%x,%x,%x,%x,%x,%x\n",
      buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6]);
  if ((hidReportLen > 0) && (hidReportLen < sizeof(hidReport))) {
    Serial1.write(hidReport, hidReportLen);
    SerialDebug.print("1:"); SerialDebug.write(hidReport, hidReportLen);
  }
}

void serialize2(const uint8_t*buf, size_t len) {
  char hidReport[64];
  int hidReportLen;
  hidReportLen = snprintf(hidReport, sizeof(hidReport),
      "%x,%x,%x,%x,%x,%x,%x\n",
      buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6]);
  if ((hidReportLen > 0) && (hidReportLen < sizeof(hidReport))) {
    Serial2.write(hidReport, hidReportLen);
    SerialDebug.print("2:"); SerialDebug.write(hidReport, hidReportLen);
  }
}

void JoystickReportParser::Parse(HID *hid, uint32_t is_rpt_id, uint32_t len, uint8_t *buf)
{
  // Checking if there are changes in report since the method was last called
  if (memcmp(buf, oldPad, RPT_GAMEPAD_LEN) == 0) return;
  // Calling Game Pad event handler
  if (joyEvents) {
    joyEvents->OnGamePadChanged((const GamePadEventData*)buf);

    memcpy(oldPad, buf, RPT_GAMEPAD_LEN);
  }
}

/*
 * Rotating the hat clock wise start from North.
 *
 * Hat  Direction
 * 0    North
 * 1    North East
 * 2    East
 * 3    South East
 * 4    South
 * 5    South West
 * 6    West
 * 7    North West
 * 8    Center, no direction
 */
#define DPAD_N  (1<<0)  // North, Up, Forward
#define DPAD_S  (1<<1)  // South, Down, Back
#define DPAD_E  (1<<2)  // East, Right
#define DPAD_W  (1<<3)  // West, Left

void dpad(int pin_num, int value)
{
  if (value) {
    pinMode(pin_num, INPUT);
  }
  else {
    pinMode(pin_num, OUTPUT); digitalWrite(pin_num, LOW);
  }
}

void JoystickEvents::OnGamePadChanged(const GamePadEventData *evt)
{
  static uint8_t last_dpad = 0x00;
  static const uint8_t mapHat2Dpad[] = {
    DPAD_N,
    DPAD_N | DPAD_E,
    DPAD_E,
    DPAD_S | DPAD_E,
    DPAD_S,
    DPAD_S | DPAD_W,
    DPAD_W,
    DPAD_N | DPAD_W,
    0
  };
  typedef struct coords {
    uint32_t x : 10;
    uint32_t y : 10;
  } coords_t;
  static const coords_t mapHat2XY[] = {
    {512, 0},       // N
    {1023, 0},      // NE
    {1023, 512},    // E
    {1023, 1023},   // SE
    {512, 1023},    // S
    {0, 1023},      // SW
    {0, 512},       // W
    {0,   0},       // NW
    {512, 512},
  };

  XACGamePadLeftEventData XAC_Left_Joystick;
  XACGamePadRightEventData XAC_Right_Joystick;

  SerialDebug.print("X: ");
  PrintHex<uint16_t>(evt->x, 0x80);
  SerialDebug.print(" Y: ");
  PrintHex<uint16_t>(evt->y, 0x80);
  SerialDebug.print(" Hat Switch: ");
  PrintHex<uint8_t>(evt->hat, 0x80);
  if (evt->hat < sizeof(mapHat2Dpad)) {
    uint8_t new_dpad = mapHat2Dpad[evt->hat];
    uint8_t changed = new_dpad ^ last_dpad;
    if (changed & DPAD_N) dpad(2, (new_dpad & DPAD_N) == 0);
    if (changed & DPAD_S) dpad(3, (new_dpad & DPAD_S) == 0);
    if (changed & DPAD_E) dpad(4, (new_dpad & DPAD_E) == 0);
    if (changed & DPAD_W) dpad(5, (new_dpad & DPAD_W) == 0);
    last_dpad = new_dpad;
  }
  SerialDebug.print(" Twist: ");
  PrintHex<uint8_t>(evt->twist, 0x80);
  SerialDebug.print(" Slider: ");
  PrintHex<uint8_t>(evt->slider, 0x80);
  SerialDebug.print(" Buttons A: ");
  PrintHex<uint8_t>(evt->buttons_a, 0x80);
  SerialDebug.print(" Buttons B: ");
  PrintHex<uint8_t>(evt->buttons_b, 0x80);
  SerialDebug.println();

  memset((uint8_t*) &XAC_Left_Joystick, 0, sizeof(XAC_Left_Joystick));
  memset((uint8_t*) &XAC_Right_Joystick, 0, sizeof(XAC_Right_Joystick));
  XACGamePadLeftEventData *left  = &XAC_Left_Joystick;
  XACGamePadRightEventData *right = &XAC_Right_Joystick;
  left->x           = evt->x;
  left->y           = evt->y;
  left->X1          = evt->base_right_lower;
  left->X2          = evt->base_right_upper;
  left->left_stick  = evt->base_center_lower;
  left->left_bumper = evt->side_trigger;
  left->A           = evt->top_lower_left;
  left->B           = evt->top_lower_right;
  left->View        = evt->base_left_lower;
  left->Menu        = evt->base_left_upper;

  right->x = mapHat2XY[evt->hat].x;
  right->y = mapHat2XY[evt->hat].y;
  right->right_stick    = evt->base_center_upper;
  right->right_bumper   = evt->front_trigger;
  right->X              = evt->top_upper_left;
  right->Y              = evt->top_upper_right;
  serialize1((uint8_t *)&XAC_Left_Joystick, RPT_GAMEPAD_LEN);
  serialize2((uint8_t *)&XAC_Right_Joystick, RPT_GAMEPAD_LEN);
}

USBHost                 UsbH;
USBHub                  Hub(&UsbH);
HIDUniversal            Hid(&UsbH);
JoystickEvents          JoyEvents;
JoystickReportParser    Joy(&JoyEvents);

void setup()
{
  SerialDebug.begin( 115200 );
  SerialDebug.println("Start");
  Serial1.begin(115200);    // Left joystick Teensy LC
  Serial2.begin(115200);    // Right joystick Teensy LC

#if defined(ADAFRUIT_ITSYBITSY_M0)
  // Assign pins 10 & 12 SERCOM functionality
  pinPeripheral(10, PIO_SERCOM);
  pinPeripheral(12, PIO_SERCOM);
#endif

  // Dpad outputs
  dpad(2, HIGH);    // Forward, North
  dpad(3, HIGH);    // Back, South
  dpad(4, HIGH);    // Right, East
  dpad(5, HIGH);    // Left, West

  if (UsbH.Init())
    SerialDebug.println("USB host did not start.");

  delay( 200 );

  if (!Hid.SetReportParser(0, &Joy))
    ErrorMessage<uint8_t>(PSTR("SetReportParser"), 1  );
}

void loop()
{
  UsbH.Task();
}
