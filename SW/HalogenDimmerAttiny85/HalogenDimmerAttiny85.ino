//timer1 @ 64Mhz
#include <avr/sleep.h>    // Sleep Modes
#include <avr/power.h>    // Power management

#define DELAY 50 // delay used in fade in, used to start DRL
#define DUTY 30 // duty cycle for DRL
#define OUT_PIN 1   //PB1 - output for PWM signal 
#define ONOFF_PIN 2    //PB2 - detecting if ON/OFFF state + wakeup from sleep - must be PB2 because INT0
#define DISABLE_PIN 0   //PB0 - detecting turnsignal, lowbeam etc to disable PWM high beam output
#define TURNSIGNAL_DELAY 5000 //safe delay to enable back DRL's if not turnsignal is present (or lowbeams...)
#define COMMING_HOME_DELAY 30000

enum state {
  OFF, // everything is off
  ON,// DRL ON
  SLEEP, //LOWPOWER MODE
  DISABLE, //LOW BEAMS ON,TURNSIGNAL ON etc...
  COMMING_HOME //30s delay befor sleep
};


volatile uint8_t status = OFF;
//volatile uint8_t dbgstatus = ON;
volatile long disable_start_time;
volatile long offdelay_time;
volatile uint8_t set_duty = 0;

//void wakeup() {
//  sleep_disable();
//  detachInterrupt(digitalPinToInterrupt(ONOFF_PIN));
//  status = OFF;
//  //Serial.println("wake");
//}
//
//void go2sleep() {
//  //Serial.println("going to sleep");
//  sleep_enable();
//  attachInterrupt(digitalPinToInterrupt(ONOFF_PIN), wakeup, LOW);
//  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
//  delay(1000);
//  status = SLEEP;
//  //Serial.println("sleep");
//  sleep_cpu();
//}

void go2sleep ()
{
  set_duty=0;
  analogWrite(OUT_PIN, set_duty);
  PCMSK  |= bit (PCINT2);  // want pin D4 / pin 3
  GIFR   |= bit (PCIF);    // clear any outstanding interrupts
  GIMSK  |= bit (PCIE);    // enable pin change interrupts
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  ADCSRA = 0;            // turn off ADC
  power_all_disable ();  // power off ADC, Timer 0 and 1, serial interface
  sleep_enable();
  sleep_cpu();
  sleep_disable();
  power_all_enable();    // power everything back on
  GIMSK  &= ~ (1 << PCIE); // enable pin change interrupts
  status = OFF;
}  // end of goToSleep

ISR (PCINT0_vect)
{
  //  if (!digitalRead(ONOFF_PIN)) {
  //    sleep_disable();
  //    power_all_enable();    // power everything back on
  //    GIMSK  &= ~ (1 << PCIE); // enable pin change interrupts
  //    detachInterrupt(digitalPinToInterrupt(ONOFF_PIN));
  //    status = OFF;
  //  } else {
  //    go2sleep();
  //  }
}

void DRL_ON() {
  //Serial.println("drl on");
  for (set_duty = 0; set_duty < DUTY; set_duty++) {
    if (!digitalRead(DISABLE_PIN)) //disable is set while we are try to turn on DRL
    {
      disable_start_time = millis();
      status = DISABLE;
      return;
    }
    analogWrite(OUT_PIN, set_duty);
    delay(DELAY);
  }
  status = ON;
}

void DRL_OFF() {
  for (set_duty; set_duty > 0; set_duty--) {
    analogWrite(OUT_PIN, set_duty);
    delay(DELAY);
  }
  analogWrite(OUT_PIN, 0);
  Serial.println(set_duty);
  status = OFF;
}

void setup() {
  //  uncoment for 32kHz ouput
  //  TCCR1 &=~(1<<CS13);
  //  TCCR1 &=~(1<<CS12);
  //  TCCR1 &=~(1<<CS11);
  //  TCCR1 |=(1<<CS10);
  pinMode(DISABLE_PIN, INPUT_PULLUP);
  pinMode(ONOFF_PIN, INPUT_PULLUP);
  digitalWrite(OUT_PIN, LOW);
  pinMode(OUT_PIN, OUTPUT);
  digitalWrite(OUT_PIN, LOW);
  //Serial.begin(115200);
  //Serial.println("start");
  // pin change interrupt (example for D4)
}

void loop() {
  switch (status) {
    case OFF:
      //if (status != dbgstatus) Serial.println("OFF");
      //dbgstatus = status;
      //wakeded up, and goes straight to low beams = disable pi is low
      if (/*digitalRead(DISABLE_PIN) &&*/ !digitalRead(ONOFF_PIN))
        //we start drl and disable it in next run on main loop
        DRL_ON();
      else
        go2sleep();
      break;
    case ON:
      //if (status != dbgstatus) Serial.println("ON");
      //dbgstatus = status;
      if (!digitalRead(DISABLE_PIN)) {
        delay(5);//debounce
        if (!digitalRead(DISABLE_PIN)) {
          status = DISABLE;
          disable_start_time = millis();
          //      Serial.println("disable");
        }
      }
      if (digitalRead(ONOFF_PIN)) {
        delay(50);
        if (digitalRead(ONOFF_PIN)) {
          //    Serial.println("going off");
          offdelay_time = millis() + 30000;
          while ((millis() < offdelay_time) && digitalRead(ONOFF_PIN)) { //30s off delay
            /*if (!digitalRead(ONOFF_PIN)) {
              //      Serial.println("disable off");
              break;
              }*/
            delay(100);
          }
          if (digitalRead(ONOFF_PIN)) {

            go2sleep();
          }

        }
      }
      break;
    case SLEEP:
      //if (status != dbgstatus) Serial.println("SLEEP");
      //dbgstatus = status;
      //this should not happened, but in case, sleep:
      set_duty = 0;
      analogWrite(OUT_PIN, set_duty);
      //digitalWrite(OUT_PIN, LOW);
      go2sleep();
      break;
    case DISABLE:
      //if (status != dbgstatus) Serial.println("DISABLE");
      //dbgstatus = status;
      DRL_OFF();
      while ((millis() - disable_start_time) < 3000) {
        if (!digitalRead(DISABLE_PIN)) {
          delay(50);//debounce
          if (!digitalRead(DISABLE_PIN)) {
            disable_start_time = millis();
            //  Serial.println("update disable time");
          }
        }
      }
      break;

  }
}
