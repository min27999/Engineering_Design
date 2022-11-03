#include <Servo.h>

// Arduino pin assignment
#define PIN_POTENTIOMETER A3 // Potentiometer at Pin A3
#define PIN_IRSENSOR A0
#define PIN_LED 9
#define PIN_SERVO 10

#define _DUTY_MIN 553  // servo full clock-wise position (0 degree)
#define _DUTY_NEU 1476 // servo neutral position (90 degree)
#define _DUTY_MAX 2399 // servo full counter-clockwise position (180 degree)
#define _DIST_MIN 100.0
#define _DIST_MAX 250.0

#define _EMA_ALPHA 0.6
#define LOOP_INTERVAL 50   // Loop Interval (unit: msec)

Servo myservo;
unsigned long last_loop_time;   // unit: msec
float dist_ema;
float dist_prev = _DIST_MAX;

void setup()
{
  myservo.attach(PIN_SERVO); 
  myservo.writeMicroseconds(_DUTY_NEU);
  pinMode(PIN_POTENTIOMETER, INPUT);
  pinMode(PIN_IRSENSOR, INPUT);
  pinMode(PIN_LED, OUTPUT);
  Serial.begin(57600);
}

void loop()
{
  unsigned long time_curr = millis();
  int a_value, duty;
  float dist_raw;

  // wait until next event time
  if (time_curr < (last_loop_time + LOOP_INTERVAL))
    return;
  last_loop_time += LOOP_INTERVAL;

  a_value = analogRead(PIN_IRSENSOR);
  dist_raw = (6762.0 / (a_value - 9) - 4.0) * 10.0 - 60.0;

  if (dist_raw < _DIST_MIN) {
    dist_raw = dist_prev;
    digitalWrite(PIN_LED, 1);       // LED OFF
  } else if (dist_raw > _DIST_MAX) {
    dist_raw = dist_prev;
    digitalWrite(PIN_LED, 1);       // LED OFF
  } else {    // In desired Range
    digitalWrite(PIN_LED, 0);       // LED ON      
    dist_prev = dist_raw;
  }
  // we need EMA filter here !!!
  dist_ema = _EMA_ALPHA * dist_raw + (1.0 - _EMA_ALPHA) * dist_ema;

  // map distance into duty
  // original
  // duty = map(a_value, 0, 1023, _DUTY_MIN, _DUTY_MAX);
  // after edit
  duty = (_DUTY_MAX - _DUTY_MIN) / (_DIST_MAX - _DIST_MIN) * (dist_ema - _DIST_MIN) + _DUTY_MIN;
  // duty = (_DUTY_MAX - _DUTY_MIN) / 1023 * a_value + _DUTY_MIN;

  myservo.writeMicroseconds(duty);

  // Serial
  Serial.print("MIN: ");      Serial.print(_DIST_MIN);
  Serial.print(", IR: ");     Serial.print(a_value);
  Serial.print(", dist: ");   Serial.print(dist_raw);
  Serial.print(", ema: ");    Serial.print(dist_ema);
  Serial.print(", servo: ");  Serial.print(duty);
  Serial.print(", MAX: ");    Serial.print(_DIST_MAX);
  // Test
  Serial.println("");

}
