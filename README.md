
## ESP32_ppm
**_ESP32_ppm_** is a library that provides **decoding and coding of PPM signals** widely used in RC transmitters and receivers equipments. It also include functionalities allowing to retrieve the main caractéristics of an unknown RC PPM signal.   
The library uses the **RMT (Remote Control Transceiver) peripheral** of the ESP32 and all output or input waveforms are managed by the hardware, requesting only a small load to the CPU.  
Specifically designed for the ESP32 family it has been tested on ESP32, ESP32C3, ESP32S3 and should run on all the processors of the family. (ESP32 Arduino core 3.2.0)   
Depending on the ESP32 model, up to 8 PPM streams can be managed on the same processor.  
 ESP32: 8 streams (RX or TX); ESP32C3: 2 RX, 2 TX; ESP32S3: 4 RX, 4 TX.  
 The library can handle PPM frames with up to 16 channels but usual RC PPM signals use less channels. A high number of channels may require some specific timing tuning. ([see Fine tunning chapiter](#tuning))  
## Usage: 
### Include  
`#include <ESP32_ppm.h>`    
### Transmitting PPM Encoded Signals 
**ppmWriter(optional_polarity)**   
Create an oject for the generation of a PPM stream.   
Polarity must be RISING or FALLING . (**default RISING**)  

`ppmWriter monPPM_TX; // create a PPM stream, with positive rising edge start impulsion`   

**begin(tx_Pin, nbr_TX_Channels )**  
Initialize the PPM stream generated on GPIO tx_pin, with nbr_TX_Channels channels. (**default 8**).
The generation of the pulses must be started effectively with **start()** or **startTest()**.   
The method returns a pointer to an array of integers where the user updates the values of each channel in microseconds (us).  
Returns NULL if an error occurs.
```
int* ppmArrayTX;
ppmArray = monPPM_TX.begin(19);  // 8 channels ppm stream on pin 19
…
ppmArray[1] = 1200;	//	value for channel 1
ppmArray[2] = 1300;	//	value for channel 2
…
```
Channel numbering is from 1 to n and index 0 of the array is not used.  
The new value is immediately used for the next ppm frame.
No checks are made upon the values of the ppm channels. (usual values are between 1000 and 2000 us …)  
**start()**   
start the PPM generation.    
**startTest(incr)**	  
start the PPM generation for test purpose: 
* in each PPM frame, the value of each channel is the same.  
* This value is incremented by the value incr (default is 1) for each frame, changing from 1000us to 1999us and so on.

**stop()**  
  stop the PPM generation. It can be resumed with **start()** or **starTest()**  
**end()**  
  stop the PPM generation and free resources. The PPM stream must be reinitialized with a **begin(….)** before use.  
**sentFrame()**   
return a long unsigned int  time stamp (in us) of the last sent frame (value of micros() ) and reset the value to 0.  
Return 0 if the application already queries **sentFrame()** for the same frame.  
This function can be used to synchronize the application if need be.

### Receiving PPM Encoded Signals
**ppmReader ()**   
Create a PPM object stream for decoding an incoming stream of ppm frames.  

`ppmReader monPPM_RX; // create an incoming  PPM stream for decoding`  

**begin(rx_Pin)**   
Initialize the PPM stream decoder on GPIO rx_pin.
 Decoding of the pulses must be started effectively with **start()**.  
The method returns a *pointer to an array of integers* where the user can read the values of each channel in microeconds (us).  
 Returns NULL if an error occurs.  
Channel numbering is from 1 to n. 
Index 0 of the array gives the number of decoded channels in the current frame.
```
int* ppmArrayRX;
ppmArray = monPPM_RX.begin(19); // receiving frame on pin 19
monPPM_RX.start();
…
for (int i =1; i< ppmArray[0];i++) Serial.printf(“%d  “, ppmArray[i]);
…
```
Update of this array is done asynchronously in respect of the main user task and will always reflect the last received values.   

**start()**  
start the PPM decoding.  
**stop()**  
stop the PPM generation. It can be resumed with **start()**.  
**end()**  
stop the PPM decoding and free resources. The PPM stream must be reinitialized with a **begin(….)** before use.   
**newFrame()**  
returns a *long unsigned int*  time stamp (in us) of the last received frame (value of micros() ) and reset the value to 0.  
Return 0 if the application already queries **newFrame()** for the same frame.  
This function can be used to synchronize the application if need be.  See example.


### Analyzing PPM Encoded Signals
**ppmSpy ()**   
Create a PPM object for analyzing an incoming stream of ppm frames.  

`ppmSpy monPPM_SPY; // create a PPM stream analyzer`

**begin(spy_Pin)**   
Initialize the PPM stream analyzer on GPIO spy_pin.
 Analyze must be started effectively with **start()**.  
The method returns a *pointer to data stucture of type*   `result_ppmSpy_t` where the user can retrieve some  characteristics of the PPM stream  (number of channels, polarity, timing ...)    
Returns NULL if an error occurs.  
This structure is populated at the end of the analysis.  
```
typedef struct {
  unsigned int polarity; // polarity of the PPM signal : RISING or FALLING
  unsigned int minLow;   // minimum duration in us of the low pulses
  unsigned int maxLow;   // maximum duration in us of the low pulses
  unsigned int minHigh;  // minimum duration in us of the high pulses
  unsigned int maxHigh;  // maximum duration in us of the high pulses
  unsigned int minFrame; // minimum duration in us of a full frame (period)
  unsigned int maxFrame; // maximum duration in us of a full frame (period)
  unsigned int minChan;  // minimum number of channels found in a frame
  unsigned int maxChan;  // minimum number of channels found in a frame
} result_ppmSpy_t;
```
**start(nbrMillis)**  
start the PPM analysis during nbrMillis milliseconds. Default is 1000ms.   
The analysis in run in background and  asynchronously of the user task and the user **must** use **doneSpy()** in order to detect the end of the anlysis.  See example.  
1000ms is usually more than enough to perform the analysis.   
Longer time may be requested if the user want to exercise the sticks of it RC equipment and retreive the min/max values reached by the PPM pulses.  
For a "normal" RC ppm signal, the minChan/manChan should be identical, as well as the minLow/maxLow in case of a FALLING signal or minHigh/maxHigh in case of a RISING signal.
Small discrepencies may come from the accuracy of the spy ...   
**stop()**   
stop the PPM analysis. It can be resumed with **start()**.  
This function should not be used in usual case: the analysis will stop anyway after nbrMillis given in the start() function.   
**end()**  xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx   
stop the PPM analysis and free resources. The PPM analyzer must be reinitialized with a **begin(….)** before use.   
**doneSpy()**  
Returns a *long unsigned int*  time stamp (in us) of the end of the test (value of micros() ) and reset the value to 0.  
Return 0 if the application already queries **doneSpy()** after the end of the analysis.
The user MUST call this function and wait on a "true" return in order to detect the end of the analysis and retrieve the characteristics of the 
PPM stream.  
```
 xxx
xxxx
xxx

}
xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
```

<a name="tuning"> 
 
## Fine tunning  

Usual RC PPM signal timing is with a start pulse (300us) and an overall frame length arround 20ms (frequency 50hz). Channel values are between 1000us and 2000us.  
By default the library uses this “standard” and it should satisfy most users.   
Nevertheless fine tuning is available and the user can change some specific values through public variables of the class. No checks are done in the library and bad values can give unpredictable results and even crashes.  

<img src="./image/waveform.JPG" title="Typical PPM frame" width="500">  

All values are in microseconds (us).  
## For encoder: 
**TX_pulse_width**   
length of the positive or negative pulse used for channel end/start (**default 300us**).  
**TX_minimum_space**  
minimum space between 2 frames. (**default 4000**).  
**TX_minimum_frame**  
minimum frame length. Define the frequency of the frame (**default 22500us**).
This value may be increased on a frame by frame basis if the encoding of the channels values plus the **TX_minimum_space** does not fit in the alloted **TX_minimum_frame**.  
This ensure that the decoder will always have enough time to detect the end of a frame. 
## For receiver:  
**RX_minimum_space**  
for the decoder, a level longer than this value will be interpreted as the end of a frame (**default 3200us**).  
***Warning***: For ESP32, ESP32S2, the **maximum value is 6553us**.   
For ESP32C3, ESP32S3, ESP32C6, ESP32P4 and ESP32H2, the **maximum value is 3276us**.  
No checks are done in the library and bad values can give unpredictable results and even crashes.  
   
These values **MUST** be changed before the initilization of the stream object (so before begin (…) )  
```
  ppmWriter myPPM_TX;
...
  myPPM_TX.TX_pulse_width = 200;
  myPPM_TX.TX_minimum_space = 4500;
  myPPM_TX.TX_minimum_frame = 15000;
...
```

