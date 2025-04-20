#ifndef PPM_READER
#define PPM_READER
#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/rmt_rx.h"
#include "driver/rmt_tx.h"

#define gpioDebug 18

#define PPM_RESOLUTION_HZ 10000000 // 10MHz resolution, 1 tick = 100ns   0.1us
//#define TICK_PER_US  10
#define TICK_PER_US  (PPM_RESOLUTION_HZ/1000000)

#define MAX_PPM_CHANNELS_RX 16   // max nbr for reader. for fun ...channel numbered from 1 to ...
#define MAX_PPM_CHANNELS_TX 16   // max nbr for transmit. for fun ...channel numbered from 1 to ...Real default
#define DEFAULT_NBR_CHANNELS_TX 4
#define DEFAULT_NBR_CHANNELS_RX 16

#define TX_MINIMUM_FRAME  22500  //TX_MINIMUM_FRAME  us
#define TX_MINIMUM_SPACE 3500 //TX_MINIMUM_SPACE (end of frame) us. Minimum idle space
#define TX_PULSE_WIDTH  300 //TX_PULSE_WIDTH   start pulse (RISING or FALLING) for a channel us

#define RX_MINIMUM_SPACE 3500 // if a level is longer than this, it is the end-of-frame marker. us

#define PPM_FRAME_LENGTH (22500*TICK_PER_US)
//#define PPM_PULSE_LENGTH (300*TICK_PER_US)

static const char *TAG = "ppmESP32";
typedef struct {
  int nbr_RX_Channels ;
  int RX_minimum_space ;    //RX_MINIMUM_SPACE 3500
  rmt_channel_handle_t rx_channel ;
  QueueHandle_t RX_queue = NULL;
  // default RAM block size is 48 for ESP32S3, C3, and 64 for ESP32
  rmt_symbol_word_t raw_symbols[48]; // 48 symbols more than enough for ppm stream
 //  unsigned long timeFrame = 0;  //
  rmt_receive_config_t receive_config = {
    // the following timing requirement is based on PPM protocol
    .signal_range_min_ns = 2000,     //  shortest pulses will be ignored
    .signal_range_max_ns = 1234567, // Initialized after. the longest duration for PPM signal. After: end frame marker
  };
  int RX_Channels_values[MAX_PPM_CHANNELS_RX + 1];  // [0]: nbr of received channels; [1]: channel1, [2]: channels 2 in us
  // we always reserve gthe ax for RX_Channels_values ... even if not used
} RX_private_area_t;

typedef struct {
  bool ppmGoOn = false;
  bool testMode = false;
  int TX_minimum_frame;   //TX_MINIMUM_FRAME in RMT ticks
  int TX_minimum_space;   //TX_MINIMUM_SPACE (end of frame)in RMT ticks
  int TX_pulse_width ;  //TX_PULSE_WIDTH   start pulse (RISING or FALLING for a channel) in RMT ticks
    QueueHandle_t TX_queue = NULL;
  int cnt = 0;  // used in test mode.
  int nbr_TX_Channels;
  int polarity;
  rmt_channel_handle_t tx_channel = NULL;
  rmt_encoder_handle_t ppm_encoder = NULL;
  rmt_transmit_config_t tx_config ;
  int TX_Channels_values[MAX_PPM_CHANNELS_TX + 1]; // [0]: not used, [1]: channel1 in us, [2]:channel2
} TX_private_area_t;

class ppmWriter {
  public:
    //   ppmWriter(void);
    ppmWriter(int polarity = RISING); // RISING , FALLING
    int *begin(uint8_t txPin, uint8_t nbr_TX_Channels = DEFAULT_NBR_CHANNELS_TX);
    void end();
    void start(void);
    void startTest(void);
    void stop(void);
    unsigned long sentFrame();
    int TX_minimum_frame = TX_MINIMUM_FRAME;   //TX_MINIMUM_FRAME 20000.0 (50hz)
    int TX_minimum_space = TX_MINIMUM_SPACE;   //TX_MINIMUM_SPACE (end of frame)
    int TX_pulse_width = TX_PULSE_WIDTH  ;  //TX_PULSE_WIDTH   start pulse (RISING or FALLING for a channel)
  private:
    TX_private_area_t *_TX_private;
    int _polarity;
    //   bool ppmEventCallBack(rmt_channel_handle_t tx_chan, const rmt_tx_done_event_data_t*edata, void *user_ctx );
    //   static size_t ppm_encode(const void *data, size_t data_size, size_t symbols_written,
    //                           size_t symbols_free, rmt_symbol_word_t *symbols, bool *done, void *arg);
    //    size_t ppm_encode(const void *data, size_t data_size, size_t symbols_written,
    //                      size_t symbols_free, rmt_symbol_word_t *symbols, bool *done, void *arg);
};
//static size_t ppm_encode(const void *data, size_t data_size, size_t symbols_written,
//                        size_t symbols_free, rmt_symbol_word_t *symbols, bool *done, void *arg);
bool ppmEventCallBack(rmt_channel_handle_t tx_chan, const rmt_tx_done_event_data_t*edata, void *user_ctx );
size_t ppm_encode(const void *data, size_t data_size, size_t symbols_written,
                  size_t symbols_free, rmt_symbol_word_t *symbols, bool *done, void *arg);
class ppmReader {
  public:
    //   ppmReader (void);
    ppmReader ();
    void end();
    int *begin(uint8_t rxPin, uint8_t nbr_RX_Channels = DEFAULT_NBR_CHANNELS_RX);
    unsigned long newFrame();
    int RX_minimum_space = RX_MINIMUM_SPACE;
  private:
    RX_private_area_t  *_RX_private;
};
//bool rmt_rx_done_callback_t example_rmt_rx_done_callback(rmt_channel_handle_t rx_channel, const rmt_rx_done_event_data_t *edata, void *user_ctx);

bool  example_rmt_rx_done_callback(rmt_channel_handle_t rx_channel, const rmt_rx_done_event_data_t *edata, void *user_ctx);

#endif
