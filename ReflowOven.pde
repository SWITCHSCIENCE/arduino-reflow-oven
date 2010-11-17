#include <LiquidCrystal.h>

/*
 *
 * Config
 *
 */
#define BUTTON 3
#define HEATER 9
#define SENSOR 10
LiquidCrystal lcd(2, 8, 4, 5, 6, 7);


/*
 *
 * Temperature sensor
 *
 */
#include "SPI.h"
const static double TEMP_ERROR = 10000;

void
sensorSetup()
{
  SPI_Master.begin(SENSOR);
}

double
sensorValue()
{
  SPI_Master.enable(SENSOR);
  int value;
  value = SPI_Master.read() << 8;
  value |= SPI_Master.read();
  SPI_Master.disable();

  if ((value & 0x0004) != 0)
    return TEMP_ERROR;
  return (value >> 3) * 0.25;
}


/*
 *
 * Main
 *
 */

/* PID */
const double Kp = 300.0;
const double Ki = 200.0;
const double Kd = 0.0;
double previous, target, integral;

/* Profile */
const double Rstart = 3.0;		// max ramp-up rate to Ts_min
const double Rup = 3.0;			// max ramp-up rate from Ts_max to Tpeak
const double Rdown = -6.0;		// max ramp-down rate from Tpeak to Ts_max
const double Ts_min = 150.0;
const double Ts_max = 190.0;
const int ts = 120;			// pre-heat duration
const double Tpeak = 232.0;
const double TL = 220.0;
const int tL = 50;			// keep above TL for tL
const double Tend = 80.0;

/* State Machine */
int state;
unsigned long nextOff, nextCheck, meltCount;
double slope, destination;
int timer;

/* Message string */
const char* messages[] = {
  /* 0 */ "Press to start",
  /* 1 */ "Ramp up",
  /* 2 */ "Pre-heat",
  /* 3 */ "Heat up",
  /* 4 */ "Melted",
  /* 5 */ "Cool down 1",
  /* 6 */ "Cool down 2"
};


void
setup()
{
  digitalWrite(HEATER, false);
  pinMode(HEATER, OUTPUT);
  digitalWrite(BUTTON, true); // pull-up
  pinMode(BUTTON, INPUT);

  Serial.begin(38400);
  sensorSetup();

  lcd.begin(16, 2); // cols, rows
  lcd.clear();
  lcd.print("Reflow Oven");
  delay(2000);
  lcd.clear();
  lcd.print(messages[0]);

  state = 0;
  nextCheck = 0;
}

/*
 * 0: waiting for button press.
 * 1: ramp-up to 150, slope rate between 1.0/sec and 3.0/sec.
 * 2: preheat to 190, for 60 and 120 seconds.
 * 3: heat to 232
 * 4: keep 232, over 220 for 30 to 60 seconds.
 * 5: cool down to 190, slope rate less than 6.0/sec.
 * 6: cool down to under 50
 */
void
loop()
{
  unsigned long now;

  /* state 0: wait for the start button to be pressed. */
  if (state == 0) {
    if (digitalRead(BUTTON) == HIGH) {
      now = millis();
      if (now < nextCheck)
        return;
      nextCheck = now + 1000;
      lcd.clear();
      lcd.print(messages[state]);
      lcd.setCursor(0, 1);
      lcd.print(sensorValue());
      lcd.print(0xdf, BYTE);
      lcd.print('C');
      return;
    }

    /* pressed */
    delay(100);
    while (digitalRead(BUTTON) == LOW)
      delay(100);
    delay(100);

    while ((previous = sensorValue()) == TEMP_ERROR)
      ;
    nextCheck = 0; // check immediately
    timer = 0;
  }

  /* Current time */
  now = millis();

  /*
   * Heater control until the next check.
   */
  if (now < nextCheck) {
    /* PWM on 1Hz */
    digitalWrite(HEATER, (now < nextOff));
    return;
  }

  /*
   * Check once a second
   */
  ++timer;
  nextCheck += 1000; // 1 second
  double temperature = sensorValue();
  if (temperature == TEMP_ERROR)
    temperature = previous;

  /* Check if the state should be changed. */
  switch (state) {
  case 0:
    state = 1;
    integral = 0;
    target = temperature;
    slope = Rstart;
    destination = Ts_min;
    break;
  case 1:
    // ramp-up to 150, slope rate between 1.0/sec and 3.0/sec.
    if (temperature >= Ts_min) {
      // preheat to 190, for 60 and 120 seconds.
      state = 2;
      integral = 0;
      target = temperature;
      slope = (Ts_max - Ts_min) / ts;
      destination = Ts_max;
    }
    break;
  case 2:
    // preheat to 190, for 60 and 120 seconds.
    if (temperature >= Ts_max) {
      // heat to 232
      state = 3;
      integral = 0;
      target = temperature;
      slope = Rup;
      destination = Tpeak;
    }
    break;
  case 3:
    // heat to 232
    if (temperature >= TL) {
      // keep 232 for 30 to 60 seconds.
      state = 4;
      integral = 0;
      target = destination = temperature;
      slope = 0;
      meltCount = 0;
    }
    break;
  case 4:
    // keep 232 for 30 to 60 seconds.
    if (++meltCount > tL) {
      // cool down to 190
      state = 5;
      integral = 0;
      target = temperature;
      slope = Rdown;
      destination = Ts_max;
    }
    break;
  case 5:
    // cool down to 190
    if (temperature <= Ts_max) {
      // cool down to under 50
      state = 6;
      integral = 0;
      target = destination = -273.0;
      slope = 0;
    }
    break;
  case 6:
    // cool down to under 50
    if (temperature <= Tend) {
      // done
      state = 0;
    }
    break;
  }

  /* Next target */
  target += slope;
  if (slope > 0 ? (target > destination) : (target < destination))
    target = destination;

  /* Heater control value */
  double error = target - temperature;
  integral += error;
  long MV = Kp * error + Ki * integral + Kd * (temperature - previous - slope);
  MV = min(max(0, MV), 1000);
  nextOff = now + MV;
  previous = temperature; // backup for the next calcuration.

  /* LCD display */
  lcd.clear();
  lcd.print(messages[state]);
  lcd.setCursor(0, 1);
  lcd.print(timer);
  lcd.print(' ');
  lcd.print(MV);
  if (temperature != TEMP_ERROR) {
    lcd.print(' ');
    lcd.print(temperature);
    lcd.print(0xdf, BYTE);
    lcd.print('C');
  }
  
  /* Report to PC */
  Serial.print(millis() / 1000);
  Serial.print(",");
  Serial.print((int)(target * 100));
  Serial.print(",");
  Serial.print((int)(temperature * 100));
  Serial.println(",");
}
