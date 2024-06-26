#include "functions.h"
#include "config.h"

#include <wiring.c>
#include <avr/io.h>

#define SERIAL_DEBUGGING_ENABLE 1

extern volatile uint8_t flag_1s;
extern volatile uint8_t seconds;
// timer 2 used for 1s flag
void timer2_init(void) { 
  TCCR2B |= (1<<CS22) | (1<<CS21) | (1<<CS20);        // prescaler 1024
  TIMSK2 |= (1<<TOIE2);                   // overflow interrupt enable
}


void readButtonState(uint8_t buttonPin, uint8_t *buttonStateReturn, uint8_t *buttonFlag) {
  static unsigned long currentMillis;
  static unsigned long previousButtonMillis;
  static unsigned long buttonPressDuration;
  static unsigned long buttonLongPressMillis;
  int minButtonLongPressDuration = 2000;
  static uint8_t buttonStatePrevious;
  static uint8_t buttonStateLongPress;
  uint8_t intervalButton = 50; 

  
  static uint8_t buttonState;
  currentMillis = millis();
  //funkcja jednorazowo zwraca 1, gdy wykryje short press i 2 gdy wykryje long press, każda z instrukcji if wykona się tylko 1 
  
  //interwał pomiędzy przyciśnięciami musi być większy niż 50ms
  if (currentMillis - previousButtonMillis > intervalButton) {
    //odczytaj stan przycisku
    buttonState = !(digitalRead(buttonPin)); //neguję odczyt, aby podczas wciśnięcia buttonState = HIGH, przy konfiguracj INPUT_PULLUP wciśnięcie przycisku = LOW

    //sprawdzenie czy przycisk został wciśnięty AND czy przycisk nie był wciśnięty wcześniej AND czy nie mierzymy już czasu długiego wciśnięcia
    if (buttonState == HIGH && buttonStatePrevious == LOW && !buttonStateLongPress) {
      buttonLongPressMillis = currentMillis;
      buttonStatePrevious = HIGH;
      *buttonFlag = 1;
      #if (SERIAL_DEBUGGING_ENABLE == 1)
      Serial.println("button pressed");

      #endif

    }
    //jak długo wciśnięty jest przycisk
    buttonPressDuration = currentMillis - buttonLongPressMillis; // przy przy pierwszym wejściu i naciśnięciu przycisku = 0

    //sprawdzanie:
    //czy przycisk jest wciśnięty AND
    //czy jest pomiar czasu na long press???? AND
    //czy czas wciśnięcia przycisku jest dłuższy nić treshhold minButtonLongPressDuration
    if (buttonState == HIGH && !buttonStateLongPress && buttonPressDuration >= minButtonLongPressDuration) {
      buttonStateLongPress = true;
      
      #if (SERIAL_DEBUGGING_ENABLE == 1)
      Serial.println("Button long press");
      #endif
      *buttonStateReturn = 2;
    }

    //release the button - kasujemy flagi i wracamy na początek 
    if (buttonState == LOW && buttonStatePrevious == HIGH) {
      buttonStatePrevious = LOW;
      buttonStateLongPress = false;
      *buttonFlag = 0;
      #if (SERIAL_DEBUGGING_ENABLE == 1)
      Serial.println("Button released");
      #endif
      //shortpress
      if (!buttonStateLongPress && buttonPressDuration < minButtonLongPressDuration) {
        *buttonStateReturn = 1;
        //*buttonFlag = 0;
        #if (SERIAL_DEBUGGING_ENABLE == 1)
        Serial.println("Button pressed shortly"); 
        #endif 
      }
    }
    
    previousButtonMillis = currentMillis;
  }
  //else *buttonStateReturn = 0;
}
void distance_init(void) {
  pinMode(DISTANCE_TRIGGER, OUTPUT);
  pinMode(DISTANCE_ECHO, INPUT);
}

//basic measurements, no filtration, return distance in [cm]
uint16_t measure_distance(void) {

  long time; //variable used to measure time of reflection
  //measurement trigger - DISTANCE_TRIGGER active for at least 10us
  digitalWrite(DISTANCE_TRIGGER, LOW);
  delayMicroseconds(2);
  digitalWrite(DISTANCE_TRIGGER, HIGH);
  delayMicroseconds(10);
  digitalWrite(DISTANCE_TRIGGER, LOW);

  time = pulseIn(DISTANCE_ECHO, HIGH, 20000); //impulse duration test
  return time / 58; //conversion to cm - based on time needed for sound to travel 1cm  

}

//  limit range to 5 - 200 cm and calulate weighted mean to smoothen results
uint16_t f_measure_distance(void) {
  static uint16_t meanDistance;
  uint16_t distance = measure_distance();

  if (distance < 5) distance = 5;
  if (distance > 200) distance = 200;
  
  meanDistance = meanDistance * KFIL;
  meanDistance = meanDistance + distance;
  meanDistance = meanDistance / (1+KFIL);
  
  return meanDistance;
}
// blink LED on standby, 
void blinkLED(uint8_t chargeLevel) {
  //setting up LED color depending on charge level
  uint8_t red, green, blue;
  if (chargeLevel > 30) {
    red = 0;
    green = 127;
    blue = 0;
  }
  else if (chargeLevel <= 30 && chargeLevel > 20) {
    red = 127;
    green = 65;
    blue = 0;
  }
  else if (chargeLevel <= 20) {
    red = 127;
    green = 0;
    blue = 0;
  }
  stripLED.clear();
  stripLED.setPixelColor(0, stripLED.Color(red, green, blue)); //trun on the 1st LED for 50ms
  stripLED.show();
  delay(50);
  stripLED.setPixelColor(0, stripLED.Color(0, 0, 0));
  stripLED.show();
}
void clear_level(void) {
        stripLED.clear();
        stripLED.show();
}
void display_level(uint8_t ledNumber, uint8_t luminosity, char color) {
  
  /*
  function for displaying valuse on the LED strip
  ledNumber - value, eg. voltage or distance, mapped to number of LEDs
  luminosity - brightness in 0-100[%] 
  color - character to set predefined color:
  'r' = red
  'g' = green
  'b' = blue
  'w' = white
  other = ~25% white

  */

  uint8_t luminosity8bit = map(luminosity, 0, 100, 0, 255); //function takes 0-100% values, this line transform that into 8bit  values
  uint8_t red, green, blue; //colors
  
  switch (color) {
    case 'r': //red
      red = luminosity8bit;
      green = 0;
      blue = 0;
      break;
    case 'g': //green
      red = 0;
      green = luminosity8bit;
      blue = 0;
      break;
    case 'b': //blue
      red = 0;
      green = 0;
      blue = luminosity8bit;
      break;
    case 'w': //white
      red = luminosity8bit;
      green = luminosity8bit;
      blue = luminosity8bit;
      break;
    default:
      red = 64;
      green = 64;
      blue = 64;
      break;
  }
  //loop iterating for each individual ledID to turn it on or off
  for (uint8_t ledID=0; ledID < LED_LENGTH; ledID++) {
    if (ledID < ledNumber) {
      
      stripLED.setPixelColor(ledID, stripLED.Color(red, green, blue));
    }
    else {
      stripLED.setPixelColor(ledID, stripLED.Color(0,0,0));
    }
  }
  stripLED.show();
}


uint16_t test_supply_voltage(uint16_t *adcReading, uint8_t *chargeLevel, float *currentVoltage) {
 
  *adcReading  = analogRead(VCC_TEST);
  
  // dividerVoltage = adcReadBuffer * (5.0 / 1024.0);
  *currentVoltage = (map(*adcReading, ADC_VCC_MIN, ADC_VCC_MAX, 1100, 1260))/100.0;
  
  int tmpChargeLevel = map(*adcReading, ADC_VCC_MIN, ADC_VCC_MAX, 0, 100);
  
  if (tmpChargeLevel < 0) *chargeLevel = 0;
  else if (tmpChargeLevel > 100) *chargeLevel = 100;
  else *chargeLevel = tmpChargeLevel; 
}


/*
// interrupt to set flag every 1s
ISR(TIMER2_OVF_vect) {
  static uint8_t cnt;
  cnt++;
  if(cnt>=31) {
    flag_1s = 1;
    seconds++;
    if(seconds>59) seconds=0;
    cnt = 0;
  }
}*/