#ifndef FUNCTIONS_H
#define FUNCTIONS_H
#include <Arduino.h> 
#include <Adafruit_NeoPixel.h> //addressable LED strip library

extern Adafruit_NeoPixel stripLED;

void distance_init(void);
uint16_t f_measure_distance(void);
uint16_t measure_distance(void);
void readButtonState(uint8_t buttonPin, uint8_t *buttonStateReturn);
void timer2_init(void);

void display_level(uint8_t ledNumber, uint8_t luminosity, char color);
void blinkLED(uint8_t chargeLevel);
void clear_level(void);
uint16_t test_supply_voltage(uint16_t *adcReading, uint8_t *chargeLevel, float *currentVoltage);

#endif