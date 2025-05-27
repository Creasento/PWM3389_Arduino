/*
  ESP32 Dual Sensor Mouse - Base code

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


#include "USBCDC.h"
#include "SPI.h"
#include "PMW3389.hpp"
#include "AdvMouse.hpp"
#include "USB.h"
#include "USBHID.h"

// User define values
#define DEFAULT_CPI 800
#define SENSOR_DISTANCE 50.83  // in mm

#define PIN_MOSI 12
#define PIN_MISO 11
#define PIN_SCLK 13
#define PIN_SS1 9  // chip select 1
#define PIN_SS2 10  // chip select 2

#define NUMBTN 3        // number of buttons attached
#define BTN1 4          // left button pin
#define BTN2 5          // right button pin
#define BTNH 8          // wheel button pin
#define WHEA 2          // wheel encoder pin A (interrupt)
#define WHEB 3          // wheel encoder pin B (interrupt)
#define DEBOUNCE  10    // debounce itme in ms. Minimun time required for a button to be stabilized.

#define MAX_CPI  16000

#define MOUSE_PRESS(x)    AdvMouse.press_(x)
#define MOUSE_RELEASE(x)  AdvMouse.release_(x)

USBHID HID;
USBCDC CDCSerial;

int posRatio = 50;      // located at 55% (centerd but slightly at the front side)


// button pins & debounce buffers
int btn_pins[NUMBTN] = { BTN1, BTN2, BTNH };
char btn_keys[NUMBTN] = { MOUSE_LEFT, MOUSE_RIGHT, MOUSE_MIDDLE };
bool btn_state[NUMBTN] = { false, false, false };
uint8_t btn_buffers[NUMBTN] = {0xFF, 0xFF, 0xFF};

// encoder pins
volatile int encoderPos = 0;  // a counter for the dial
int scrollStep = 1;
uint8_t encA_buffer = 0xFF;
uint8_t encB_buffer = 0xFF;
uint8_t lastEncoded = 0;

PMW3389 sensor1, sensor2;
unsigned long lastTS;                 // Last timestamp for sensor reading
unsigned long lastButtonCheck = 0;

float remain_dx, remain_dy;
bool lastNA = false;

float sensor_dist_inch = (float)SENSOR_DISTANCE / 25.4;
int current_cpi = DEFAULT_CPI;
float cpi_divider = (float)MAX_CPI / DEFAULT_CPI;

// extern USBCDC Serial;
AdvMouseHIDDevice AdvMouse;
SPIClass spi;

void setup() {
  pinMode(PIN_SS1, OUTPUT);
  pinMode(PIN_SS2, OUTPUT);

  digitalWrite(PIN_SS1, HIGH);
  digitalWrite(PIN_SS2, HIGH);

  // Serial.begin(115200); // Set baud rate to 115200
  // Serial.begin();
  AdvMouse.begin();
  USB.begin();
  HID.begin();
  CDCSerial.begin(115200);  // CDC 포트 활성화
  CDCSerial.println("HID + CDC 동시 활성화됨!");
  spi.begin(PIN_SCLK, PIN_MISO, PIN_MOSI, -1); // Note: CS pin is not defined here because we manage it manually
  spi.setHwCs(false); // Disable hardware CS pin management

  // Begin Serial
  //Serial.begin(921600);
  CDCSerial.println("Serial begin");

  if (sensor2.begin(PIN_SCLK, PIN_MISO, PIN_MOSI, PIN_SS2, MAX_CPI, &spi))
    CDCSerial.println("sensor 2 init done");
  else
    CDCSerial.println("sensor 2 init FAILED");


  if (sensor1.begin(PIN_SCLK, PIN_MISO, PIN_MOSI, PIN_SS1, MAX_CPI, &spi))
    CDCSerial.println("sensor 1 init done");
  else
    CDCSerial.println("sensor 1 init FAILED");


  CDCSerial.println(sensor1.getCPI());   // verifying that set CPI is correct
  CDCSerial.println(sensor2.getCPI());

  setSoftCPI(DEFAULT_CPI);
  buttons_init();
  encoder_init();

  // clear the previous buffers
  lastTS = micros();
  sensor1.readBurst();
  sensor2.readBurst();

  CDCSerial.println("init done");
}

void loop() {
  unsigned long currentTS = micros();
  unsigned long elapsed = currentTS - lastTS;

  check_buttons_state();
  check_encoder_state();

  if (elapsed > 700)
  {
    lastTS = currentTS;

    PMW3389_DATA data1 = sensor1.readBurst();
    PMW3389_DATA data2 = sensor2.readBurst();

    // rotate data1's dx, dy by -90 deg
    int tmp = data1.dx;
    data1.dx = data1.dy;
    data1.dy = -tmp;
    // rotate data1's dx, dy by 90 deg
    tmp = data2.dx;
    data2.dx = data2.dy;
    data2.dy = -tmp;


    // Combine sensor1 and sensor2 data into --> data
    PMW3389_DATA data;

    data.isOnSurface = data1.isOnSurface && data2.isOnSurface;
    data.isMotion = data1.isMotion || data2.isMotion;

    float rat = posRatio / 100.0;

    // dx and dy are calculated in MAX_CPI. merge them using the calculated ratio (rat)
    float dx = (float)data1.dx * (1 - rat) + (float)data2.dx * rat + remain_dx;
    float dy = (float)data1.dy * (1 - rat) + (float)data2.dy * rat + remain_dy;

    data.isMotion = (dx != 0) || (dy != 0);

    // divide down to the soft CPI
    int mdx = int(dx / cpi_divider);
    int mdy = int(dy / cpi_divider);

    remain_dx = dx - mdx * cpi_divider;
    remain_dy = dy - mdy * cpi_divider;

    data.dx = mdx;
    data.dy = mdy;

    data.SQUAL = (data1.SQUAL + data2.SQUAL) / rat;

    // detected any move?
    bool moved = data1.dx != 0 || data1.dy != 0 || data2.dx != 0 || data2.dy != 0;
    moved = moved || AdvMouse.needSendReport();

    uint8_t btn_report = btn_state[0] + (btn_state[1] << 1) + (btn_state[2] << 2);


    if (AdvMouse.needSendReport() || (data.isOnSurface && moved) || encoderPos != 0)
    {
      signed char mdx = constrain(data.dx, -127, 127);
      signed char mdy = constrain(data.dy, -127, 127);

      int wheel = (encoderPos == 0) ? 0 : (encoderPos > 0 ? scrollStep : -scrollStep);

      AdvMouse.move(mdx, mdy, wheel);
    }
    encoderPos = 0; // Reset position after scrolling

    if (moved)
    {
      String logStr = String(micros()) + "\t";

      if (data1.isOnSurface)
      {
        logStr += String(data1.dx) + "\t" + String(data1.dy);
      }
      else
      {
        logStr += "NA\tNA";
      }

      logStr += "\t";

      if (data2.isOnSurface)
      {
        logStr += String(data2.dx) + "\t" + String(data2.dy);
      }
      else
      {
        logStr += "NA\tNA";
      }
      logStr += "\t";

      logStr += String(data.dx) + "\t" + String(data.dy) + "\t" + String(btn_report);

      logStr += "\t";

      logStr += String(data1.SQUAL) + "\t" + String(data2.SQUAL);

      CDCSerial.println(logStr);
      lastNA = false;
    }
    else if (!data.isOnSurface && !lastNA)
    {
      //CDCSerial.println(String(micros()) + "\tNA\tNA\tNA\tNA\tNA\tNA\tNA\tNA\tNA");
      lastNA = true;
    }
    else
    {
      //CDCSerial.println(String(micros()) + "\tNA\tNA\tNA\tNA\tNA\tNA\tNA\tNA\tNA");
    }
  }

  // handle serial commands
  // c: report current CPI
  // C: set CPI
  // P: set sensor position
  // R: report current sensor position and cpi values
  while (CDCSerial.available() > 0)
  {
    char c = CDCSerial.read();

    if (c == 'c') // report current CPI
    {
      CDCSerial.println("Current CPI: " + String(current_cpi) + "\n" + String(sensor1.getCPI()) + "\n" + String(sensor2.getCPI()));
    }
    else if (c == 'C')   // set CPI
    {
      int newCPI = readNumber();

      setSoftCPI(newCPI);
      CDCSerial.print(String(current_cpi) + "\n" + String(cpi_divider));
    }
    else if (c == 'P')  // set sensor position
    {
      int newPos = readNumber();
      posRatio = constrain(newPos, 0, 100);
      CDCSerial.println(posRatio);
    }
    else if (c == 'R') // report current values
    {
      CDCSerial.println("Pos:\t" + String(posRatio) + "\n" + "CPI:\t" + String(current_cpi));
    }
  }
}

void buttons_init()
{
  for (int i = 0; i < NUMBTN; i++)
  {
    pinMode(btn_pins[i], INPUT_PULLUP);
  }
}


// Button state checkup routine, fast debounce is implemented.
void check_buttons_state()
{
  unsigned long elapsed = micros() - lastButtonCheck;

  // Update at a period of 1/8 of the DEBOUNCE time
  if (elapsed < (DEBOUNCE * 1000UL / 8))
    return;

  lastButtonCheck = micros();

  // Fast Debounce (works with mimimal latency most of the time)
  for (int i = 0; i < NUMBTN ; i++)
  {
    int state = digitalRead(btn_pins[i]);
    btn_buffers[i] = btn_buffers[i] << 1 | state;

    // btn_buffer detects 0 when the switch shorts, 1 when opens.
    if (btn_state[i] == false &&
        (btn_buffers[i] == 0xFE || btn_buffers[i] == 0x00) )
      // 0xFE = 0b1111:1110 button pressed for the first time (for fast press detection w. minimum debouncing time)
      // 0x00 = 0b0000:0000 force press when consequent on state (for the DEBOUNCE time) is detected
    {
      MOUSE_PRESS(btn_keys[i]);
      btn_state[i] = true;
    }
    else if ( btn_state[i] == true &&
              (btn_buffers[i] == 0x07 || btn_buffers[i] == 0xFF) )
      // 0x07 = 0b0000:0111 button released consequently 3 times after stabilized press (not as fast as press to prevent accidental releasing during drag)
      // 0xFF = 0b1111:1111 force release when consequent off state (for the DEBOUNCE time) is detected
    {
      MOUSE_RELEASE(btn_keys[i]);
      btn_state[i] = false;
    }
  }
}

void encoder_init()
{
  pinMode(WHEA, INPUT_PULLUP);
  pinMode(WHEB, INPUT_PULLUP);
}

void check_encoder_state()
{
  // read WHEA and WHEB into the buffers
  uint8_t stateA = digitalRead(WHEA) != 0;
  uint8_t stateB = digitalRead(WHEB) != 0;

  encA_buffer = encA_buffer << 1 | stateA;
  encB_buffer = encB_buffer << 1 | stateB;

  // debounce using the buffer, fall: 80: 1000 0000, rise: 7F: 0111 1111
  if (encA_buffer == 0x80 || encA_buffer == 0x7F || encB_buffer == 0x80 || encB_buffer == 0x7F)
  {
    int encoded = (stateA << 1) | stateB;   // pack the events into two bits
    lastEncoded = (lastEncoded << 2) | encoded; // store the encoded value into the lastEncoded buffer

    switch (lastEncoded & 0b1111)
    {
      case 0b0111:  // B first -> A second
      case 0b1000:
        encoderPos++;
        break;
      case 0b1011:  // A first -> B second
      case 0b0100:
        encoderPos--;
        break;
    }
  }
}

void setSoftCPI(int CPI)
{
  current_cpi = constrain(CPI, 100, 16000);
  cpi_divider = (float)MAX_CPI / current_cpi;
}


unsigned long readNumber()
{
  String inString = "";
  for (int i = 0; i < 10; i++)
  {
    while (CDCSerial.available() == 0);
    int inChar = CDCSerial.read();
    if (isDigit(inChar))
    {
      inString += (char)inChar;
    }

    if (inChar == '\n')
    {
      int val = inString.toInt();
      return (unsigned long)val;
    }
  }

  // flush remain strings in serial buffer
  while (CDCSerial.available() > 0)
  {
    CDCSerial.read();
  }
  return 0UL;
}
