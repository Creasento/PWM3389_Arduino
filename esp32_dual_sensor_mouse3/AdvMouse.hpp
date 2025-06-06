/*
  AdvMouse.hpp - Library for advanced HID Mouse feature, for ESP32 core

  Copyright (c) 2024, Sunjun Kim

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/


#include "USB.h"
#include "USBHID.h"

#define MOUSE_LEFT    0x01
#define MOUSE_RIGHT   0x02
#define MOUSE_MIDDLE  0x04
#define MOUSE_BACK    0x08
#define MOUSE_FORWARD 0x10
#define MOUSE_ALL (MOUSE_LEFT | MOUSE_RIGHT | MOUSE_MIDDLE | MOUSE_BACK | MOUSE_FORWARD)

static const uint8_t report_descriptor[] = {
  0x05, 0x01,   // Usage Page (Generic Desktop)
  0x09, 0x02,   // Usage (Mouse)
  0xA1, 0x01,   // Collection (Application)
  0x85, 0x01,   // HID report ID = 1
  0x05, 0x09,   //   Usage Page (Button)
  0x19, 0x01,   //   Usage Minimum (Button #1)
  0x29, 0x05,   //   Usage Maximum (Button #5)
  0x15, 0x00,   //   Logical Minimum (0)
  0x25, 0x01,   //   Logical Maximum (1)
  0x95, 0x05,   //   Report Count (5)
  0x75, 0x01,   //   Report Size (1)
  0x81, 0x02,   //   Input (Data, Variable, Absolute)
  0x95, 0x01,   //   Report Count (1)
  0x75, 0x03,   //   Report Size (3)
  0x81, 0x03,   //   Input (Constant) // Byte 1
  0x05, 0x01,   //   Usage Page (Generic Desktop)
  0x09, 0x30,   //   Usage (X)
  0x09, 0x31,   //   Usage (Y)
  0x16, 0x00, 0x80, //   Logical Minimum (-32,768)
  0x26, 0xFF, 0x7F, //   Logical Maximum (32,767)
  0x36, 0x00, 0x80, //   Physical Minimum (-32,768)
  0x46, 0xFF, 0x7F, //   Physical Maxiumum (32,767)
  0x75, 0x10,   //   Report Size (16),
  0x95, 0x02,   //   Report Count (2),
  0x81, 0x06,   //   Input (Data, Variable, Relative) // Byte 3, 5
  0x09, 0x38,   //   Usage (Wheel)
  0x15, 0x81,   //   Logical Minimum (-127)
  0x25, 0x7F,   //   Logical Maximum (127)
  0x35, 0x81,   //   Phyiscal Minimum (-127)
  0x45, 0x7F,   //   Physical Maxiumum (127)
  0x75, 0x08,   //   Report Size (8)
  0x95, 0x01,   //   Report Count (1)
  0x81, 0x06,   //   Input (Data, Variable, Relative) // Byte 6
  0xC0      // End Collection
};

class AdvMouseHIDDevice : public USBHIDDevice {
  private:
    USBHID HID;
    uint8_t _buttons;
    bool _isReportSent;

    void buttonsWithoutMove(uint8_t b)
    {
      if (b != _buttons)
      {
        _buttons = b;
        _isReportSent = false;
      }
    }

    void buttons(uint8_t b)
    {
      if (b != _buttons)
      {
        _buttons = b;
        move(0, 0, 0);
      }
    }

  public:
    AdvMouseHIDDevice(void)
    {
      static bool initialized = false;
      if (!initialized) {
        initialized = true;
        HID.addDevice(this, sizeof(report_descriptor));
      }
    }
    void begin(void)
    {
      HID.begin();
      _isReportSent = false;
    }

    void click(uint8_t b)
    {
      _buttons = b;
      move(0, 0, 0);
      _buttons = 0;
      move(0, 0, 0);
    }

    bool needSendReport(void)
    {
      return !_isReportSent;
    }

    uint16_t _onGetDescriptor(uint8_t *buffer)
    {
      memcpy(buffer, report_descriptor, sizeof(report_descriptor));
      return sizeof(report_descriptor);
    }

    void move(int16_t x, int16_t y, int8_t wheel)
    {
      uint8_t m[6];
      m[0] = _buttons;
      m[1] = x & 0xFF;
      m[2] = (x >> 8) & 0xFF;
      m[3] = y & 0xFF;
      m[4] = (y >> 8) & 0xFF;
      m[5] = wheel;
      HID.SendReport(1, m, 6);
      _isReportSent = true;
    }


    void press(uint8_t b)
    {
      buttons(_buttons | b);
    }

    void release(uint8_t b)
    {
      buttons(_buttons & ~b);
    }

    void press_(uint8_t b)
    {
      buttonsWithoutMove(_buttons | b);
    }

    void release_(uint8_t b)
    {
      buttonsWithoutMove(_buttons & ~b);
    }

    bool isPressed(uint8_t b)
    {
      if ((b & _buttons) > 0)
        return true;
      return false;
    }
};
