#ifndef ESP32_ppm
#define ESP32_ppm
#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/rmt_rx.h"
#include "driver/rmt_tx.h"
//Warning: the driver makes some checks (linked to hardware limitation) on the values passed in the rmt_receive_config_t.
//    uint32_t filter_reg_value = ((uint64_t)rx_chan->filter_clock_resolution_hz * config->signal_range_min_ns) / 1000000000UL;
//    uint32_t idle_reg_value = ((uint64_t)channel->resolution_hz * config->signal_range_max_ns) / 1 000 000 000UL;
//The 2 values must be lower than 65535 for ESP32,S2 and 32767 for S3, C3, C6 and H2 (RMT_LL_MAX_IDLE_VALUE )
//With a 10Mhz clock resolution, maximum value for signal_range_max_ns (ie RX_MINIMUM_SPACE) is 6553 for EESP32, and S2
//and 3276 for ESP32C3,S3,C6,H2
#define PPM_RESOLUTION_HZ 10000000 // 10MHz resolution, 1 tick = 100ns   0.1us
#define TICK_PER_US  (PPM_RESOLUTION_HZ/1000000)

#define PPM_RESOLUTION_SPY_HZ   2000000 // 2MHz resolution, 1 tick = 0.5us
#define TICK_SPY_PER_US  (PPM_RESOLUTION_SPY_HZ/1000000)

#define MAX_PPM_CHANNELS_RX 16   // max nbr for reader. for fun ...channel numbered from 1 to ...
#define MAX_PPM_CHANNELS_TX 16   // max nbr for transmit. for fun ...channel numbered from 1 to ...
#define DEFAULT_NBR_CHANNELS_TX 8
#define DEFAULT_NBR_CHANNELS_RX 16

#define TX_MINIMUM_FRAME  22500  //TX_MINIMUM_FRAME  us
#define TX_MINIMUM_SPACE 4000 //TX_MINIMUM_SPACE (end of frame) us. Minimum idle space
#define TX_PULSE_WIDTH  300 //TX_PULSE_WIDTH   start pulse (RISING or FALLING) for a channel us

#define RX_MINIMUM_SPACE 3200 // if a level is longer than this, it is the end-of-frame marker. us
//3200 will be OK for all ESPs at 10Mhz

// for SPY we don't want to stop reading if there is a long level/ end-of-frame marker.
// We must use the maximum .... limited by the frequency PPM_RESOLUTION_SPY_HZ and the ESP model
#define SPY_MINIMUM_SPACE 16383ul   // In us. OK on C3 at 2Mhz (not a lot ....) and ESP32
#define SPY_MIN_IDLE 2500  // in us. idle length used by SPY to detect potential end of frame

static const char *TAG = "ppmESP32";
typedef struct {
  int RX_Channels_values[MAX_PPM_CHANNELS_RX + 1];  // [0]: nbr of received channels; [1]: channel1, [2]: channels 2 in us
  // we always reserve the max for RX_Channels_values ... even if not used
  QueueHandle_t RX_queue = NULL;
  rmt_channel_handle_t rx_channel ;
  // default RAM block size is 48 for ESP32S3, C3, and 64 for ESP32
  rmt_symbol_word_t raw_symbols[RMT_SYMBOLS_PER_CHANNEL_BLOCK]; //  more than enough for ppm stream
  rmt_receive_config_t receive_config = {
    // the following timing requirement is based on PPM protocol
    .signal_range_min_ns = 2000,     //  shortest pulses will be ignored
    .signal_range_max_ns = 1234567, // Initialized after. the longest duration for PPM signal. After: end frame marker
  };
} RX_private_area_t;
typedef struct {
  unsigned int polarity;  // polarity of the PPM signal : RISING or FOLLING
  unsigned int minLow = 12345678; // minimum duration in us of the low pulses
  unsigned int maxLow = 0; // maximum duration in us of the low pulses
  unsigned int minHigh = 12345678; // minimum duration in us of the high pulses
  unsigned int maxHigh = 0; // maximum duration in us of the high pulses
  unsigned int minFrame = 12345678; // minimum duration in us of a full frame (period)
  unsigned int maxFrame = 0; // maximum duration in us of a full frame (period)
  unsigned int minChan = 12345678; // minimum number of channels found in a frame
  unsigned int maxChan = 0; // minimum number of channels found in a frame
} result_ppmSpy_t;

typedef struct {
  unsigned int timeStopSpy;
  unsigned int lengthFrame;
  unsigned int nbrChan;
  unsigned int sumLow = 0, sumHigh = 0, nbrHigh = 0, nbrLow = 0;
  result_ppmSpy_t result_spy;
  int num_symbols;
  bool startedFrame;
  QueueHandle_t Spy_queue  = NULL;
  rmt_channel_handle_t Spy_channel ;
  // default RAM block size is 48 for ESP32S3, C3, and 64 for ESP32
  rmt_symbol_word_t raw_symbols[RMT_SYMBOLS_PER_CHANNEL_BLOCK]; //  more than enough for ppm stream
  rmt_receive_config_t receive_config = {
    // the following timing requirement is based on PPM protocol
    .signal_range_min_ns = 2000,     //  shortest pulses will be ignored
    .signal_range_max_ns = SPY_MINIMUM_SPACE * 1000ul, //  the longest duration for PPM signal. This "end frame marker" should never occurs
    .flags = {
      .en_partial_rx = true, // enable partial read for SPY, with the largest .signal_range_max_ns
    },
  };
} Spy_private_area_t;


typedef struct {
  int TX_minimum_frame;   //TX_MINIMUM_FRAME in RMT ticks
  int TX_minimum_space;   //TX_MINIMUM_SPACE (end of frame)in RMT ticks
  int TX_pulse_width ;  //TX_PULSE_WIDTH   start pulse (RISING or FALLING for a channel) in RMT ticks
  int cnt = 0;  // used in test mode.
  int incr = 1; // used intest mode
  int nbr_TX_Channels;
  int TX_Channels_values[MAX_PPM_CHANNELS_TX + 1]; // [0]: not used, [1]: channel1 in us, [2]:channel2
  int polarity;
  QueueHandle_t TX_queue = NULL;
  rmt_channel_handle_t tx_channel = NULL;
  rmt_encoder_handle_t ppm_encoder = NULL;
  rmt_transmit_config_t tx_config ;
  bool ppmGoOn = false;
  bool testMode = false;
} TX_private_area_t;

class ppmWriter {
  public:
    ppmWriter(int polarity = RISING); // RISING , FALLING
    int *begin(uint8_t txPin, uint8_t nbr_TX_Channels = DEFAULT_NBR_CHANNELS_TX);
    void end();
    void start(void);
    void startTest(int incr = 1 );
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
static bool ppmEventCallBack(rmt_channel_handle_t tx_chan, const rmt_tx_done_event_data_t*edata, void *user_ctx );
static size_t ppm_encode(const void *data, size_t data_size, size_t symbols_written,
                         size_t symbols_free, rmt_symbol_word_t *symbols, bool *done, void *arg);
class ppmReader {
  public:
    ppmReader ();
    void end();
    //   int *begin(uint8_t rxPin, uint8_t nbr_RX_Channels = DEFAULT_NBR_CHANNELS_RX);
    int *begin(uint8_t rxPin);
    void start(void);
    void stop(void);
    unsigned long newFrame();
    int RX_minimum_space = RX_MINIMUM_SPACE;
  private:
    RX_private_area_t  *_RX_private;
};

class ppmSpy {
  public:
    ppmSpy();
    result_ppmSpy_t *begin(uint8_t spyPin);
    void start(unsigned int timeSpy = 1000); // length of test in seconds
    void stop(void);
    void end();
    unsigned long doneSpy();
  private:
    Spy_private_area_t  *_Spy_private;
};
//bool rmt_rx_done_callback_t example_rmt_rx_done_callback(rmt_channel_handle_t rx_channel, const rmt_rx_done_event_data_t *edata, void *user_ctx);

static bool  example_rmt_rx_done_callback(rmt_channel_handle_t rx_channel, const rmt_rx_done_event_data_t *edata, void *user_ctx);
static bool  check_rmt_rx_done_callback(rmt_channel_handle_t rx_channel, const rmt_rx_done_event_data_t *edata, void *user_ctx);

#endif
