// (c) kovo
//hallogen dimmer

/*
 * 3.7.2024 kovo: typos, cleanup commented out code 
 */

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
  DISABLE, //LOW BEAMS ON,TURNSIGNAL ON, DRL OFF, NO SLEEP
  COMMING_HOME //30s delay before sleep
};


volatile uint8_t status = OFF;
volatile long disable_start_time;
volatile long offdelay_time;
volatile uint8_t set_duty = 0;

void go2sleep ()
{
  set_duty=0;
  analogWrite(OUT_PIN, set_duty);
  PCMSK  |= bit (PCINT2);  // want pin D4 / pin 3
  GIFR   |= bit (PCIF);    // clear any outstanding interrupts
  GIMSK  |= bit (PCIE);    // enable pin change interrupts
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  ADCSRA = 0;            // turn off ADC
  power_all_disable();  // power off ADC, Timer 0 and 1, serial interface
  sleep_enable();
  sleep_cpu();
  sleep_disable();
  power_all_enable();    // power everything back on
  GIMSK  &= ~ (1 << PCIE); // enable pin change interrupts
  status = OFF;
}  // end of goToSleep

void DRL_ON() {
  for (set_duty = 0; set_duty < DUTY; set_duty++) {
    //while we are spinning on DRL, DISABLE pin is set (we start LOW beams of exmaple) 
    // so we cut turning on DRL 
    //and imidietly jump to turning them off = fade out
    if (!digitalRead(DISABLE_PIN))
    {
      delay(50);//debounce
      if (!digitalRead(DISABLE_PIN)) {
        disable_start_time = millis();
        status = DISABLE;
        return;
      }
    }
    analogWrite(OUT_PIN, set_duty);
    delay(DELAY); //DELAY=50ms, duty 30 => rump up is 1500ms aka 1.5s
  }
  //all good, we DRL is ON
  status = ON;
}

void DRL_OFF() {
  for (set_duty; set_duty > 0; set_duty--) {
    //do not checking for pin status, this rutine is used in more then one places
    //pin DISABLE change is handled in main loop
    analogWrite(OUT_PIN, set_duty);
    delay(DELAY); //DELAY=50ms, duty 30 => rump up is 1500ms aka 1.5s
  }
  //analogWrite(OUT_PIN, 0);//should not be needed....
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
}

void loop() {
  switch (status) {
    case OFF:
      //wakeded up, and goes straight to low beams = disable pin is low
      if (!digitalRead(ONOFF_PIN)) //HW inverted, using transistor 
        //we start drl and disable it in next run in main loop
        DRL_ON(); 
      else
        go2sleep();
      break;
    case ON:
      if (!digitalRead(DISABLE_PIN)) {
        delay(50);//debounce
        if (!digitalRead(DISABLE_PIN)) {
          status = DISABLE;
        }
      }
      if (digitalRead(ONOFF_PIN)) {
        delay(50);
        if (digitalRead(ONOFF_PIN)) {
          offdelay_time = millis() + 30000;
          while ((millis() < offdelay_time) && digitalRead(ONOFF_PIN)); //wait for 30s
          go2sleep();
        }
      }
      break;
    case SLEEP:
      //this should not happened, but in case, sleep:
      analogWrite(OUT_PIN, 0);
      go2sleep();
      break;
    case DISABLE:
      disable_start_time = millis();
      DRL_OFF();
      //loop for 3seconds if DISABLE pin is not set
      //otherwise loop forever
      while ((millis() - disable_start_time) < 3000) {
        if (!digitalRead(DISABLE_PIN)) {
          delay(50);//debounce
          if (!digitalRead(DISABLE_PIN)) {
            disable_start_time = millis();
          }
        }
      }
      break;

  }
}
