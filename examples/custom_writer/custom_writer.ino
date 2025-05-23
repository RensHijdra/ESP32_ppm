/*
 * This example create a simple 4 channels PPM streams and demonstrate how
 * the timing of the frame may be changed by the user if need be.
 * ( A small USB logic analyzer is handy for the control of PPM stream.)
 */
#include <ESP32_ppm.h>
#define tx_pin 21  // Gpiofor output PPM stream

ppmWriter myPPM_TX;
int* ppmArrayTX;
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.printf("Starting a PPM writer on pin %d\n", tx_pin);
  Serial.printf("Default characteristics of the PPM pulses:\n"
                "TX_pulse_width:%dus ,TX_minimum_space:%dus , TX_minimum_frame:%dus \n",
                myPPM_TX.TX_pulse_width, myPPM_TX.TX_minimum_space, myPPM_TX.TX_minimum_frame);
  myPPM_TX.TX_pulse_width = 200;
  myPPM_TX.TX_minimum_space = 4500;
  myPPM_TX.TX_minimum_frame = 15000;
  Serial.printf("We can change these characteristics:\n"
                "TX_pulse_width:%dus ,TX_minimum_space:%dus , TX_minimum_frame:%dus \n",
                myPPM_TX.TX_pulse_width, myPPM_TX.TX_minimum_space, myPPM_TX.TX_minimum_frame);
 // Changes of the timing parfameter must be done before begin(...)
  ppmArrayTX = myPPM_TX.begin(tx_pin, 4);  // PPM frames with 4 channels ans special timing
 // initialize the first frame with some values
 for (int i =1; i<=4; i++)ppmArrayTX[i]= 1000+100*i;
  myPPM_TX.start();
  delay(100); // send this frame during 100ms
  // update some channels
  ppmArrayTX[3] = 1600;
  ppmArrayTX[4] = 1800;
  delay(200); // send this frame during 200ms
  myPPM_TX.stop(); // stop the PPM stream.
}

void loop() {
}
