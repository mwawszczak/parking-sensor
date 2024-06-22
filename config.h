
#define SERIAL_STATUS 1
#define SERIAL_VCC_STATUS 0
#define SERIAL_DISTANCE_STATUS 1

#define builtInLED 13                   // Arduino Nano built-in LED (test purpose only)

// stip LED configuration
#define LED_LENGTH 13                    // LED strip lenght
#define STRIP_LED_PIN 8                   // data pin for LED strip
#define LUMINOSITY_LOW_BAT 40           // LED strip luminosity in [%] on LOW_BAT
#define LUMINOSITY_STD 100              // standard LED strip luminosity in [%]

// buttons configuration
#define BUTTON_1 6                      // first button lamp/voltage
// power button + diode? 


// distance measturement configuration 
#define DISTANCE_TRIGGER 4               // trigger signal for distance sensor
#define DISTANCE_ECHO 5                  // echo signal for distance sensor
#define KFIL 20                          // weight for distance mean measurements
#define DISTANCE_DEADZONE 10            // distance change in [cm] that doesn't activate the active mode

// lamp configuraion 
#define LAMP_OFF_DELAY 10               // lamp mode off delay in [s]
#define INACTIVITY_TIME 15              // inactivity time in [s]

// voltage measurement configuration 
#define VOLTAGE_LOW_BAT 30              // treshhold for low bat in [%]
#define VOLTAGE_OFF_DELAY 10            // how long in [s] voltage is displayed on LED strip

#define VCC_TEST A1
#define ADC_VCC_MIN 470                 // ADC reading for low voltage 11.00 V
#define ADC_VCC_MAX 940                 // ADC reading for max voltage 12.60 V
#define VOLTAGE_SHUTDOWN 1              // if battery is equal to 1% - shutwodn
#define POWER_ON_PIN 3                  // N-MOSFET controling main power line pin