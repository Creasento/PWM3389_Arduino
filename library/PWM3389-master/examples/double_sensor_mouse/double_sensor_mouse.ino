#include <PMW3389.h>
#include <Mouse.h>

#define SS  10
#define S2  9
#define NUMBTN 2        // number of buttons attached
#define BTN1 8          // left button pin
#define BTN2 4          // right button pin
#define DEBOUNCE  10    // debounce itme in ms. Minimun time required for a button to be stabilized.

int btn_pins[NUMBTN] = { BTN1, BTN2 };
char btn_keys[NUMBTN] = { MOUSE_LEFT, MOUSE_RIGHT };

int cpiValue = 1600;

PMW3389 sensor;
PMW3389 sensor2;

bool btn_state[NUMBTN] = { false, false };
uint8_t btn_buffers[NUMBTN] = {0xFF, 0xFF};

unsigned long lastButtonCheck = 0;

void setup() {
  Serial.begin(9600);

  // With this line, your arduino will wait until a serial communication begin.
  // If you want your mouse application to work as soon as plug-in the USB, remove this line.

  //sensor.begin(10, 1600); // to set CPI (Count per Inch), pass it as the second parameter
  if (sensor.begin(SS)) // 10 is the pin connected to SS of the module.
    Serial.println("Sensor initialization successed");
  else
    Serial.println("Sensor initialization failed");
  if (sensor2.begin(S2)) // 10 is the pin connected to SS of the module.
    Serial.println("Sensor2 initialization successed");
  else
    Serial.println("Sensor2 initialization failed");
  Mouse.begin();
  sensor.setCPI(cpiValue);    // or, you can set CPI later by calling setCPI();
  sensor2.setCPI(cpiValue);
  buttons_init();
}

void loop() {
  check_buttons_state();

  PMW3389_DATA data = sensor.readBurst();
  PMW3389_DATA data2 = sensor2.readBurst();
  if (data.isOnSurface && data.isMotion && data2.isOnSurface && data2.isMotion)
  {
    int mdx = constrain(data.dx, -255, 255);
    int mdy = constrain(data.dy, -255, 255);
    int mdx2 = constrain(data2.dx, -255, 255);
    int mdy2 = constrain(data2.dy, -255, 255);
    Mouse.move((mdy - mdy2) / 16, (mdx2 - mdx) / 16, 0);
    /* you can use below code if you want to see data
    Serial.print(" sensor 1: ");
    Serial.print(data.dx);
    Serial.print("\t");
    Serial.print(data.dy);
    Serial.print(" sensor 2: ");
    Serial.print(data2.dx);
    Serial.print("\t");
    Serial.print(data2.dy);
    Serial.println();
    */
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

  // Fast Debounce (works with 0 latency most of the time)
  for (int i = 0; i < NUMBTN ; i++)
  {
    int state = digitalRead(btn_pins[i]);
    btn_buffers[i] = btn_buffers[i] << 1 | state;

    if (!btn_state[i] && btn_buffers[i] == 0xFE) // button pressed for the first time
    {
      Mouse.press(btn_keys[i]);
      btn_state[i] = true;
    }
    else if ( (btn_state[i] && btn_buffers[i] == 0x01) // button released after stabilized press
              // force release when consequent off state (for the DEBOUNCE time) is detected
              || (btn_state[i] && btn_buffers[i] == 0xFF) )
    {
      Mouse.release(btn_keys[i]);
      btn_state[i] = false;
    }
  }
}
