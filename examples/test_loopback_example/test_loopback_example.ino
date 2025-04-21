/*
 * Test in loopback mode.
 * On the same ESP we create a channel for writing a PPM stream on pin tx_pin
 * and a channel for reading on pin rx_pin. There is a jumper between both pins.
 * Please, update de #define for the pins, following your configuration.
 * The writer is started in testmode and generate PPM frames where all channels have the
 * same values. This value is incremented for each frame and varies between 1000 and 1999.
 * We check that the frames are received in increasing order, with no lost frame and wrong channel values.
 */

#include <ESP32_ppm.h>

#define rx_pin 19
#define tx_pin 21

int* ppmArray;
int* ppmArrayTX;

ppmReader myPPM_RX;
ppmWriter myPPM_TX;

int expectedVal = 0;
int nbrReceived = 0;
void checkFrame();

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.printf("Starting loopback test example. Put jumper between pin %d et pin %d\n", rx_pin, tx_pin);

  ppmArray = myPPM_RX.begin(rx_pin);
  ppmArrayTX = myPPM_TX.begin(tx_pin);  // create a 8 channels PPM writer, RISING style 
  myPPM_TX.startTest();
  myPPM_RX.start();
  // just for information, we print the first 5 frames received, with the timestamp
  int cnt = 0;
  do {
    unsigned long ts = myPPM_RX.newFrame();  // the time stamp of the frame
    if (ts) {
      cnt++;
      Serial.printf("%u ", ts);
      for (int i = 1; i <= ppmArray[0]; i++) Serial.printf(" % d  ", ppmArray[i]);
      Serial.println();
    }
  } while (cnt < 5);
}

void loop() {
  if ( myPPM_RX.newFrame()) checkFrame();
}

// check if the frame are in sequence, with no lost frame or wrong channel values
void checkFrame() {
  nbrReceived++;
  // test values are between 1000 and 1999. Each channel in a frame have the same value. Increase by 1 at each frame
  for (int i = 1; i <= ppmArray[0] ; i++) {
    if (ppmArray[i] != expectedVal ) {
      Serial.printf(" ------ err seq. Frames received:%d, Channel:%d, Value:%d, Expected:%d",
                    nbrReceived, i, ppmArray[i], expectedVal);
      Serial.println();
      break;
    }
  }
  expectedVal = ppmArray[1]  + 1; // assuming that channel 1 is not too bad ...
  if  ( expectedVal > 1999) {
    Serial.printf(" +++ Frames received:%d\n", nbrReceived);
    expectedVal = 1000;
  }
}
