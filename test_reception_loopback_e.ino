// Développement pour une librairie ESP21 PPM reader et writer
// test_reception_loopback_d
//    - rmtSetEOT vu dans exemple RMT_EndOfTra...
//    - rajout de   . Meme gestion que pour RX: queue avec retour de Micros
// test_reception_loopback_d
//    - version avec une queue pour communiquer la reception frame
//    - OK mais on perd un peu de temps dans ISR (2us) et pour interroger la queue( 1 à 2)
// test_reception_loopback_c
//    - Semble OK même avec ppmReader::newFrame() basé sur micros() mais le pb de race existe surement.
//    - essais en revenant à ppmReader::newFrame() basé sur micros()
// test_reception_loopback_b
//   - essais avec monPPM_RX.newFrame() faisant seulement true / false
//    - problème de race:  semble un peut reglé dans ppmReader::newFrame(). Course avec l'ISR ..
// test_reception_loopback_a
//    - TX et RX sur même ESP
//    - parfois on ne detecte pas une nouvelle frame. Pb de micros() unsigned int ?? long ??
// test_reception_a
//    - pour tester la sequence de frame générée en mode TX test, au niveau du main.
//        On perd des trames   (on en perd pas dans la call back ...)
// decodePPM_development_c
//    - basé sur generatePPM_development_c
//    -  start / stop / pour RX et TX ??
//    - end() rajouté pour RX et TX.
//    + attention, en reception en [0] on a un canal en plus
//    - on perd qq trame si test dans le main ??
//    - essayer un loop back, sur le même ESP32

#include "ppmESP32.h"
#define gpioDebug2 5
#define gpioDebug3 22
int* ppmArray;
int* ppmArrayTX;
ppmReader monPPM_RX;
ppmWriter monPPM_TX;

bool stamp = false;

int expectedVal = 0;
int nbrReceived = 0;
void modeTest() {
  bool err = false;
  digitalWrite(gpioDebug3, !digitalRead(gpioDebug3));
  nbrReceived++;
  // test values are between 1000 and 1999. Each channel in a frrame have the same value. Increase by 1 at each frame
  for (int i = 1; i <= ppmArray[0] ; i++) {
    if (ppmArray[i] != expectedVal ) {
      Serial.printf(" ------ err seq main. Frames received:%d, Channel:%d, Value:%d, Expected:%d",
                    nbrReceived, i, ppmArray[i], expectedVal);
      Serial.println();
      err = true;
      break;
    }
  }
  if (!err) digitalWrite(gpioDebug3, !digitalRead(gpioDebug3));
  expectedVal = ppmArray[1]  + 1; // assuming that [1] is not too bad ...
  if  ( expectedVal > 1999) {
    Serial.printf(" +++main. Frames received:%d\n", nbrReceived);
    expectedVal = 1000;
  }
}
void check(int *lepointer, int pin) {
  Serial.printf("pin: %d ", pin);
  if (lepointer == NULL)
    Serial.println (" bad !!");
  else
    Serial.println ("ok");
}
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  delay(2000);

  pinMode(gpioDebug, OUTPUT);
  digitalWrite(gpioDebug, LOW);
  pinMode(gpioDebug2, OUTPUT);
  digitalWrite(gpioDebug2, LOW);
  pinMode(gpioDebug3, OUTPUT);
  digitalWrite(gpioDebug3, LOW);

  ppmArray = monPPM_RX.begin(19, 8);
  ppmArrayTX = monPPM_TX.begin(21);
  check(ppmArrayTX, 21);
  check(ppmArray, 19);
  Serial.println("Fin setup.");
  monPPM_TX.startTest();
  // monPPM_TX.start();
}

void loop() {
  digitalWrite(gpioDebug2, !digitalRead(gpioDebug2));
  // "anything that is not 0 is true."

  if ( monPPM_RX.newFrame() ) {
    modeTest();
  }
  unsigned long ii;
  ii = monPPM_TX.sentFrame();
//  if (ii != 0)Serial.printf("RX sent:%u\n", ii);
}
/*
  delay(1);
  if (millis() > 4000) {
  Serial.println("Appel end()");
  monPPM_RX.end();

  Serial.println("C'est fait");
  while (true) delay(1);
  }
*/
/*
  for (int i = 1; i< ppmArray[0];i++){
  Serial.printf(" %d ",ppmArray[i]);
  }
  Serial.println();
  delay(12345);
*/
