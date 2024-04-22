#define ADVANCE_MODE

#include <SPI.h>
#include <avr/pgmspace.h>
#include "PMW3389.h"

#ifdef ADVANCE_MODE
#include "AdvMouse.h"
#define MOUSE_BEGIN       AdvMouse.begin()
#define MOUSE_PRESS(x)    AdvMouse.press_(x)
#define MOUSE_RELEASE(x)  AdvMouse.release_(x)
#else
#include <Mouse.h>
#define MOUSE_BEGIN       Mouse.begin()
#define MOUSE_PRESS(x)    Mouse.press(x)
#define MOUSE_RELEASE(x)  Mouse.release(x)
#endif

// User define values
#define DEFAULT_CPI  600
#define SENSOR_DISTANCE 50.83  // in mm

#define MAX_CPI  16000

#define SS1  10          // Slave Select pin. Connect this to SS on the module. (Front sensor)
#define SS2  9         // Slave Select pin. Connect this to SS on the module. (Rear sensor)
#define NUMBTN 3        // number of buttons attached
#define BTN1 4          // left button pin
#define BTN2 8          // right button pin
#define BTNH 5          // wheel button pin
#define WHEA 2          // wheel encoder pin A (interrupt)
#define WHEB 3          // wheel encoder pin B (interrupt)
#define DEBOUNCE  10    // debounce itme in ms. Minimun time required for a button to be stabilized.

int btn_pins[NUMBTN] = { BTN1, BTN2, BTNH };
char btn_keys[NUMBTN] = { MOUSE_LEFT, MOUSE_RIGHT, MOUSE_MIDDLE };

#define PINA_BIT (1 << PD1) // Pin 2 on Arduino Micro
#define PINB_BIT (1 << PD0) // Pin 3 on Arduino Micro

// Don't need touch below.
PMW3389 sensor1, sensor2;
int posRatio = 50;      // located at 55% (centerd but slightly at the front side)

// button pins & debounce buffers
bool btn_state[NUMBTN] = { false, false, false };
uint8_t btn_buffers[NUMBTN] = {0xFF, 0xFF, 0xFF};

// internal variables
unsigned long lastTS;
unsigned long lastButtonCheck = 0;

float remain_dx, remain_dy;

bool reportSQ = false;  // report surface quality
bool lastNA = false;

float sensor_dist_inch = (float)SENSOR_DISTANCE / 25.4;
int current_cpi = DEFAULT_CPI;
float cpi_divider = (float)MAX_CPI / DEFAULT_CPI;

//wheel part

// usually the rotary encoders three pins have the ground pin in the middle
enum PinAssignments {
  encoderPinA = 2,   // right
  encoderPinB = 3,   // left
  clearButton = 8    // another two pins
};

volatile int encoderPos = 0;  // a counter for the dial
volatile unsigned int lastScrollStableState = 0;
volatile unsigned int lastScrollTransitState = 0;
int scrollStep = 1;

// interrupt service routine vars

void setup() {
  Serial.begin(9600);
  //while(!Serial);
  //sensor.begin(10, 1600); // to set CPI (Count per Inch), pass it as the second parameter

  sensor2.begin(SS2);
  sensor1.begin(SS1);
  delay(250);

  if (sensor1.begin(SS1)) // 10 is the pin connected to SS of the module.
    Serial.println("Sensor1 initialization successed");
  else
    Serial.println("Sensor1 initialization failed");

  if (sensor2.begin(SS2)) // 10 is the pin connected to SS of the module.
    Serial.println("Sensor2 initialization successed");
  else
    Serial.println("Sensor2 initialization failed");

  sensor1.setCPI(MAX_CPI);    // or, you can set CPI later by calling setCPI();
  sensor2.setCPI(MAX_CPI);    // or, you can set CPI later by calling setCPI();

  // sensor1.writeReg(REG_Control, 0b11000000); // turn 90 deg configuration
  // sensor2.writeReg(REG_Control, 0b11000000);

  // sensor1.writeReg(REG_Lift_Config, 0b11);  // set lift detection height = norminal height + 3 mm
  // sensor2.writeReg(REG_Lift_Config, 0b11);

  int cpi1 = sensor1.getCPI();
  int cpi2 = sensor2.getCPI();

  if (cpi1 != cpi2 || cpi1 != MAX_CPI)
    Serial.println("WARNING: CPI initialization failed");

  remain_dx = remain_dy = 0.0;

  setSoftCPI(DEFAULT_CPI);

  MOUSE_BEGIN;
  buttons_init();

  // clear the previous buffers
  lastTS = micros();
  sensor1.readBurst();
  sensor2.readBurst();

  //wheel part
  pinMode(encoderPinA, INPUT);
  pinMode(encoderPinB, INPUT);
  pinMode(clearButton, INPUT);
  // turn on pullup resistors
  digitalWrite(encoderPinA, HIGH);
  digitalWrite(encoderPinB, HIGH);
  digitalWrite(clearButton, HIGH);

  // encoder pin on interrupt 0 (pin 2)
  attachInterrupt(0, doEncoder, CHANGE);
  // encoder pin on interrupt 1 (pin 3)
  attachInterrupt(1, doEncoder, CHANGE);
}


bool sent = false;
void loop() {
  unsigned long currentTS = micros();
  unsigned long elapsed = currentTS - lastTS;

  check_buttons_state();

  if (elapsed > 1993)
  {
    lastTS = currentTS;

    PMW3389_DATA data1 = sensor1.readBurst();
    PMW3389_DATA data2 = sensor2.readBurst();

    // rotate data1's dx, dy by -90 deg
    int tmp = data1.dx;
    data1.dx = -data1.dy;
    data1.dy = tmp;
    // rotate data1's dx, dy by 90 deg
    tmp = data2.dx;
    data2.dx = -data2.dy;
    data2.dy = tmp;


    PMW3389_DATA data;

    data.isOnSurface = data1.isOnSurface && data2.isOnSurface;
    data.isMotion = data1.isMotion || data2.isMotion;

    float rat = posRatio / 100.0;

    // dx and dy are calculated in MAX_CPI
    float dx = (data1.dx + data2.dx) / rat + remain_dx;
    float dy = (data1.dy + data2.dy) / rat + remain_dy;

    data.isMotion = (dx != 0) || (dy != 0);

    // divide down to the soft CPI
    int mdx = int(dx / cpi_divider);
    int mdy = int(dy / cpi_divider);

    remain_dx = dx - mdx * cpi_divider;
    remain_dy = dy - mdy * cpi_divider;

    data.dx = mdx;
    data.dy = mdy;

    data.SQUAL = (data1.SQUAL + data2.SQUAL) / rat;

    //data.dx = data1.dx + data2.dx;
    //data.dy = data1.dy + data2.dy;

    bool moved = data1.dx != 0 || data1.dy != 0 || data2.dx != 0 || data2.dy != 0;

#ifdef ADVANCE_MODE
    moved = moved || AdvMouse.needSendReport();
#endif

    byte btn_report = btn_state[0] + (btn_state[1] << 1);


    if (moved)
    {
      Serial.print(micros());
      Serial.print('\t');

      if (data1.isOnSurface)
      {
        Serial.print(data1.dx);
        Serial.print('\t');
        Serial.print(data1.dy);
      }
      else
      {
        Serial.print("NA\tNA");
      }

      Serial.print('\t');

      if (data2.isOnSurface)
      {
        Serial.print(data2.dx);
        Serial.print('\t');
        Serial.print(data2.dy);
      }
      else
      {
        Serial.print("NA\tNA");
      }
      Serial.print('\t');

      Serial.print(data.dx);
      Serial.print('\t');
      Serial.print(data.dy);

      Serial.print('\t');
      Serial.println(btn_report);
      lastNA = false;
    }
    else if (!data.isOnSurface && !lastNA)
    {
      Serial.print(micros());
      Serial.println("\tNA\tNA\tNA\tNA\tNA\tNA\tNA");
      lastNA = true;
    }


#ifdef ADVANCE_MODE
    if (AdvMouse.needSendReport() || (data.isOnSurface && moved) || encoderPos != 0)
    {
      //if (AdvMouse.needSendReport() && !data.isMotion)
      //  Serial.println("Btn report");
      int wheel = (encoderPos == 0) ? 0 : (encoderPos > 0 ? scrollStep : -scrollStep);
      AdvMouse.move(data.dx, data.dy, wheel);
    }
    encoderPos = 0; // Reset position after scrolling
#else
    if (data.isOnSurface && moved || encoderPos != 0)
    {
      signed char mdx = constrain(data.d  x, -127, 127);
      signed char mdy = constrain(data.dy, -127, 127);

      int wheel = (encoderPos == 0) ? 0 : (encoderPos > 0 ? scrollStep : -scrollStep);

      Mouse.move(mdx, mdy, wheel);
    }
    encoderPos = 0; // Reset position after scrollings
#endif

    if (reportSQ && data.isOnSurface) // print surface quality
    {
      Serial.println(data.SQUAL);
    }
  }

  // command process routine
  while (Serial.available() > 0)
  {
    char c = Serial.read();

    if (c == 'Q') // Toggle reporting surface quality
    {
      reportSQ = !reportSQ;
    }
    else if (c == 'c') // report current CPI
    {
      Serial.print("Current CPI: ");
      Serial.println(current_cpi);
      Serial.println(sensor1.getCPI());
      Serial.println(sensor2.getCPI());
    }
    else if (c == 'C')   // set CPI
    {
      int newCPI = readNumber();

      setSoftCPI(newCPI);
      Serial.print(current_cpi);
      Serial.print('\t');
      Serial.println(cpi_divider);
    }
    else if (c == 'P')  // set sensor position
    {
      int newPos = readNumber();
      posRatio = constrain(newPos, 0, 100);
      Serial.println(posRatio);
    }
    else if (c == 'R') // report current values
    {
      Serial.print("Pos:\t");
      Serial.println(posRatio);
      Serial.print("CPI:\t");
      Serial.println(current_cpi);
    }
  }
}

// s1_dx, s1_dy => sensor 1 (=front) (dx, dy)
// s2_dx, s2_dy => sensor 2 (=rear) (dx, dy)
// current_cpi  =>
void translate_virtual_sensor(int s1_dx, int s1_dy, int s2_dx, int s2_dy, float weight_x, float weight_y, float &vs_pos_x, float &vs_pos_y)
{
  int vx = s2_dx - s1_dx;
  int vy = s2_dy - s1_dy;

  float d = sensor_dist_inch * (float)current_cpi; // inter-sensor distance in counts

  float theta = (float)vx / d; // value is in radian.

  float cos_t = cos(theta);
  float sin_t = sin(theta);

  float sa_x = d * weight_x;
  float sa_y = d * weight_y;

  vs_pos_x = (float)s2_dx + cos_t * sa_x - sin_t * sa_y - sa_x;
  vs_pos_y = (float)s2_dy + sin_t * sa_x + cos_t * sa_y - sa_y;
}

void buttons_init()
{
  for (int i = 0; i < NUMBTN; i++)
  {
    pinMode(btn_pins[i], INPUT_PULLUP);
  }
}

void setSoftCPI(int CPI)
{
  current_cpi = constrain(CPI, 100, 12000);
  cpi_divider = (float)MAX_CPI / current_cpi;
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


unsigned long readNumber()
{
  String inString = "";
  for (int i = 0; i < 10; i++)
  {
    while (Serial.available() == 0);
    int inChar = Serial.read();
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
  while (Serial.available() > 0)
  {
    Serial.read();
  }
  return 0UL;
}

//wheel part
// state: 00: 0 / 01: 1 / 10: 2 / 11: 3
//  wheel up:
//   0 0 > 0 1 > 1 1   (=0-1-3)  or   1 1 > 1 0 > 0 0 (=3-2-0)
//  wheel down:
//   0 0 > 1 0 > 1 1   (=0-2-3)  or   1 1 > 0 1 > 0 0 (=3-1-0)
void doEncoder() {
  bool encA = (PIND & PINA_BIT) != 0;
  bool encB = (PIND & PINB_BIT) != 0;
  int scrollState = encA | encB << 1; // state: 00: 0 / 01: 1 / 10: 2 / 11: 3

  int scroll = 0;
  if (lastScrollStableState != scrollState && scrollState == 0) scroll = lastScrollTransitState == 1 ? -1 : 1; // 3->?->0
  if (lastScrollStableState != scrollState && scrollState == 3) scroll = lastScrollTransitState == 1 ? 1 : -1; // 0->?->3

  encoderPos -= scroll;

  if (scrollState == 0 || scrollState == 3) lastScrollStableState = scrollState;
  if (scrollState == 1 || scrollState == 2) lastScrollTransitState = scrollState;
}
