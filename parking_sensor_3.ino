#define TEST_LED 13
#include <Adafruit_NeoPixel.h> //addressable LED strip library
#include "functions.h"
#include "config.h"

unsigned long hundrethSec;

volatile uint8_t flag_1s;
volatile uint8_t flag_01s;
volatile uint8_t hundrethsSeconds;
volatile uint8_t seconds;
unsigned long startTimestamp;
unsigned long longestLoopTime;
unsigned long loopTime;
uint8_t lampCounter;
uint8_t lampFlag;

volatile uint8_t button1Flag;
volatile uint8_t button2Flag;
volatile uint8_t buttonSelect = 1;
volatile uint8_t button1State;
volatile uint8_t button2State;

uint16_t distance;
uint16_t previousDistance;
uint8_t inactivityTimer;          // inactivity timer to quit active mode

uint16_t ADCReading;              // VCC ADC reading
uint8_t chargeLevel;              // battery charge level 0-100
volatile float currentVoltage;             // curent supply voltage 
uint8_t lowBatFlag;               // flag is set, when supply voltage drops below VOLTAGE_LOW_BAT value
uint8_t voltageCounter;           // counter of seconds - do quit voltage test mode in determined time

Adafruit_NeoPixel stripLED(LED_LENGTH, STRIP_LED_PIN, NEO_GRB + NEO_KHZ800); //instance of Adafruit_NeoPixel obeject created
uint8_t luminosity;                // current luminosity depending on lowBatFlag

// enum for modes
enum modes {standby, active, lamp, voltageTest};
modes mode = standby; //creates var in type of modes and assign value of standbyMode



void setup() {
  pinMode(TEST_LED, OUTPUT);
  //sei();
  //timer2_init(); 
  distance_init(); 
  Serial.begin(9600);
  
  // initiate stripLED
  stripLED.begin();
  stripLED.clear();

  //startup animation
  for (uint8_t i=0; i < LED_LENGTH; i++) {
    stripLED.setPixelColor(i, stripLED.Color(0, 255, 0));
    stripLED.show();
    delay(50);
    stripLED.clear();
    //stripLED.setPixelColor(i, stripLED.Color(0, 0, 0));
    stripLED.show();
  }
  for (uint8_t i=LED_LENGTH; i > 0; i--) {
    stripLED.setPixelColor(i, stripLED.Color(0, 255, 0));
    stripLED.show();
    delay(50);
    stripLED.clear();
    //stripLED.setPixelColor(i, stripLED.Color(0, 0, 0));
    stripLED.show();
  }

  // power control pin
  pinMode(POWER_ON_PIN, OUTPUT);    
  digitalWrite(POWER_ON_PIN, HIGH);         // power supply on    

  pinMode(TEST_LED, OUTPUT);
  pinMode(BUTTON_1, INPUT_PULLUP);
  pinMode(BUTTON_2, INPUT_PULLUP);
  
}

void loop() {
  
  startTimestamp = millis();
  if (startTimestamp - hundrethSec > 100) {
    flag_01s = 1;
    hundrethsSeconds++;
    
    //Serial.print("hudnreths: ");
    //Serial.println(hundrethsSeconds);
    if (hundrethsSeconds > 9) {
      
      
      hundrethsSeconds = 0;
      flag_1s = 1;
      seconds++;
    if(seconds>59) seconds=0;
    }
    
    hundrethSec = millis();
  }

  
  // read button state
//readButtonState(BUTTON_1, &button1State, &button1Flag);

  if (buttonSelect == 1) {
    readButtonState(BUTTON_1, &button1State, &button1Flag);
    
    if (!button1Flag) buttonSelect = 2;
  }
  else if (buttonSelect = 2) {
    readButtonState(BUTTON_2, &button2State, &button2Flag);
    if (!button2Flag) buttonSelect = 1;
  }
 
  
  // measure distance 
  // distance = f_measure_distance();

  if (button1State == 1) mode = lamp;
  else if (button1State == 2);

  if (button2State == 1) mode = voltageTest;
  else if (button2State == 2);  
  
  // change mode if button short or long press is detected
  //if (button1State == 1 ) mode = lamp;
  //else if (button1State == 2 ) mode = voltageTest;
  
  
  
  //distance = 100;
   
  // standby mode
  if(mode == standby) {
    
    // blink LED every 5 seconds, color will change depending on charge level
    if (seconds % 5 == 0 && flag_1s) blinkLED(chargeLevel);
    // check difference between distance and previousDistance
    // if larger than 10 cm - go to active mode
    if (abs(int(previousDistance - distance) ) >  DISTANCE_DEADZONE) mode = active; 
    

  }
  
  // active mode - measure distance and show on the LED 
  if(mode == active) {
    

    // show on LED
    uint8_t mappedDistance = map(distance, 0, 200, 1, LED_LENGTH-1);
    if (distance > 10) display_level(mappedDistance, luminosity , 'g');
    else {
      if (hundrethsSeconds % 2 == 0) {
        display_level(LED_LENGTH, luminosity, 'r');
        
      }
      else clear_level();
      
    }
    
    // inactivity timer - 
    if ( (abs(int(previousDistance - distance))) < DISTANCE_DEADZONE ) {            // type casting to int as difference of 2 unsigned ints will be unsigned int 
      if (flag_1s) inactivityTimer++;
    }
    else inactivityTimer = 0;
    
    if (inactivityTimer > INACTIVITY_TIME) {
      inactivityTimer = 0;
      clear_level();
      mode = standby;
    }

  }
  
  // lamp mode 
if (mode == lamp) {
    
    if (button1State == 1 && lampFlag == 0) {
      lampFlag = 1;
      display_level(LED_LENGTH, luminosity, 'w');
      //delay(200);
    #if SERIAL_STATUS == 1 
      Serial.println("lamp ON");
    #endif
      button1State = 0;
    } 

    else if (button1State == 1 && lampFlag == 1) {
      lampFlag = 0;
      display_level(0, luminosity, 'w');

      
      mode = standby;
    #if SERIAL_STATUS == 1
      Serial.println("lamp OFF");
    #endif
      lampCounter = 0;      //
      button1State = 0; 
    }
    
    // auto turn off after LAMP_OFF_DELAY
    if (flag_1s) lampCounter++;
    
    if (lampCounter > LAMP_OFF_DELAY) {
      lampCounter = 0;
      lampFlag = 0;
      stripLED.clear();
      stripLED.show();
      mode = standby;
    }
    

  }

 
  // voltage test mode
  
  if (mode == voltageTest) {
    
    
    
    // show voltage on LED strip
    uint8_t mappedChargeLevel = map(chargeLevel, 0, 100, 0, LED_LENGTH);
    display_level(mappedChargeLevel, luminosity, 'g');
    // count 10s
    if (flag_1s) {
      voltageCounter++;
      if (voltageCounter > VOLTAGE_OFF_DELAY){
        stripLED.clear();
        stripLED.show();
        voltageCounter = 0;
        mode = standby;

      }
    }

    // reset counter
  }
  
  if (flag_01s) flag_01s = 0;

  // actions performed every 1s no matter what is the current mode
  if (flag_1s) {
    
    // save previousDistance every 5s
    if ((seconds %3 == 0)) {
      
      previousDistance = distance;
    }

    
    
  
    // test voltage
    // if the voltage drops below VOLTAGE_LOW_BAT value - set lowBatFlag and reduce LED luminosity to LUMINOSITY_LOW_BAT value
    // 2 if is kind of hysteresis to reduce LED blinkig while the voltage slightly change on the threshhold
    test_supply_voltage(&ADCReading, &chargeLevel, &currentVoltage);
    float tempVoltage = map(ADCReading, ADC_VCC_MIN, ADC_VCC_MAX, 1100, 1260);
    if (chargeLevel < VOLTAGE_LOW_BAT) {
      lowBatFlag = 1;
      luminosity = LUMINOSITY_LOW_BAT;
    }
    if (chargeLevel > VOLTAGE_LOW_BAT + 3) {
      lowBatFlag = 0;
      luminosity = LUMINOSITY_STD;
    }
    

    // if voltage is critically low - auto shutdown
    if (chargeLevel <= 1) digitalWrite(POWER_ON_PIN, LOW);      // auto shut down 
    // clear flag_1s
    
    flag_1s = 0; 

    #if SERIAL_STATUS == 1
    //Serial.print("seconds: ");
    //Serial.println(seconds);
    Serial.print("Mode: ");
    switch (mode) {
      case standby:
        Serial.println("standby");
        break;
      case lamp:
        Serial.println("lamp");
        break;
      case voltageTest:
        Serial.println("voltageTest");
        break;
      case active:
        Serial.println("active");
        break;
    }
    //Serial.print("lampFlag: ");
    //Serial.println(lampFlag);
    #if SERIAL_VCC_STATUS == 1
    Serial.print("ADC reading: ");
    Serial.println(ADCReading);
    Serial.print("Current Voltage: ");
    Serial.println(currentVoltage);
    Serial.print("charge level: ");
    Serial.println(chargeLevel);
    #endif

    #if SERIAL_DISTANCE_STATUS == 1
    Serial.print("Distance: ");
    Serial.println(distance);
    Serial.print("Previous Distance: ");
    Serial.println(previousDistance);
    Serial.print("Diff: ");
    Serial.println(abs(int(previousDistance - distance)));
    
    #endif
    //Serial.print("voltageCounter: ");
    //Serial.println(voltageCounter);
    






    #endif
    Serial.print("Loop Time: ");
    Serial.println(loopTime);
    Serial.print("seconds: ");
    Serial.println(seconds);
    Serial.println();
  }


  // clear buttons' state  
  
  button1State = 0;
  button2State = 0; 
  loopTime = millis() - startTimestamp;
  if (longestLoopTime < loopTime) longestLoopTime = loopTime;
  //Serial.println(loopTime);
}


