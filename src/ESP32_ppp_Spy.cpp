#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/rmt_rx.h"
#include "ESP32_ppm.h"
static bool  check_rmt_rx_done_callback(rmt_channel_handle_t rx_channel, const rmt_rx_done_event_data_t *edata, void *user_ctx);

#if !SOC_RMT_SUPPORT_RX_PINGPONG
#pragma message "Warning: ppmSpy functionality of the ESP32_ppm library not supported with this SoC"
#endif
// SPY for anlyzing a "standard RC ppm stream"

ppmSpy::ppmSpy() {
}

result_ppmSpy_t *ppmSpy::begin(uint8_t spyPin)
{
#if !SOC_RMT_SUPPORT_RX_PINGPONG
  return NULL;   // we need to use  ".en_partial_rx = true" in rmt_receive_config_t receive_config.
  // (not available on all the ESP. Bad for ESP32 ... OK for ESP32C3,C5,C6 etc ...
#endif
  ESP_LOGI(TAG, "create RMT RX channel for Spy");
  rmt_rx_channel_config_t rx_channel_cfg = {
    .gpio_num   = static_cast<gpio_num_t> (spyPin),
    .clk_src = RMT_CLK_SRC_DEFAULT,
    .resolution_hz = PPM_RESOLUTION_SPY_HZ,
    .mem_block_symbols = RMT_SYMBOLS_PER_CHANNEL_BLOCK,   // amount of RMT symbols that the channel can store at a time
    // (mini 64 for ESP32, 48 for ESP32C3 or S3)
  };
  // data for communication with the callback
  _Spy_private = new (Spy_private_area_t );
  if (_Spy_private == NULL) return NULL;
  ESP_ERROR_CHECK(rmt_new_rx_channel(&rx_channel_cfg, &_Spy_private->Spy_channel));
  ESP_LOGI(TAG, "register RX done callback for check");
  rmt_rx_event_callbacks_t cbs = {
    .on_recv_done =  check_rmt_rx_done_callback,  // de type rmt_rx_done_callback_t
  };
  ESP_ERROR_CHECK(rmt_rx_register_event_callbacks(_Spy_private->Spy_channel, &cbs, _Spy_private));
  _Spy_private->Spy_queue = xQueueCreate(1, sizeof(unsigned long));
  assert( _Spy_private->Spy_queue);

  return  &_Spy_private->result_spy;
}  // end of beginCheck

void ppmSpy::start(unsigned int timeSpy) {
  // lenght of the test in seconds
  ESP_LOGI(TAG, "Enable RMT SPY channel");
  _Spy_private->timeStopSpy = millis() + timeSpy;
  ESP_ERROR_CHECK(rmt_enable(_Spy_private->Spy_channel));
  // start T
  ESP_ERROR_CHECK(rmt_receive(_Spy_private->Spy_channel, _Spy_private-> raw_symbols,
                              sizeof(_Spy_private->raw_symbols), &_Spy_private->receive_config)) ;

}
void ppmSpy::stop(void) {
  ESP_ERROR_CHECK(rmt_disable(_Spy_private->Spy_channel));
}
void ppmSpy::end() {
  if (_Spy_private != NULL) {
    rmt_disable(_Spy_private->Spy_channel);
    rmt_del_channel(_Spy_private->Spy_channel);
    vQueueDelete(_Spy_private->Spy_queue);
    delete _Spy_private;
    _Spy_private = NULL;
  }
}
unsigned long ppmSpy::doneSpy() {
  // "anything that is not 0 is true."
  unsigned long ii;
  if (xQueueReceive(_Spy_private->Spy_queue, &ii, 0)) return ii;
  return 0;
}

//void ppmSpy::manageFrame(unsigned int length) {
static void IRAM_ATTR manageFrame(unsigned int lengthInTicks, Spy_private_area_t *_Spy_private) {
  unsigned int length =   lengthInTicks / TICK_SPY_PER_US;  // length in us
  if ( length > SPY_MIN_IDLE) {
    // detection of idle. Idle is at the end of the curent pulse
    if (_Spy_private->startedFrame) {
      // end of a frame. We already see the idle of a previous frame
      _Spy_private->lengthFrame = _Spy_private->lengthFrame + length;
      _Spy_private->nbrChan++;
      _Spy_private->nbrChan = _Spy_private->nbrChan / 2 - 1;
      _Spy_private->result_spy.minFrame = min(_Spy_private->result_spy.minFrame, _Spy_private->lengthFrame);
      _Spy_private->result_spy.maxFrame = max(_Spy_private->result_spy.maxFrame, _Spy_private->lengthFrame);
      _Spy_private->result_spy.minChan = min(_Spy_private->result_spy.minChan, _Spy_private->nbrChan);
      _Spy_private->result_spy.maxChan = max(_Spy_private->result_spy.maxChan, _Spy_private->nbrChan);
      _Spy_private->lengthFrame = 0;
      _Spy_private->nbrChan = 0;
    } else {
      // first idle that we find in the array of pulse
      //  We start a new frame: compute the number of channels
      //  and the total frame length
      _Spy_private->startedFrame = true;
      _Spy_private->lengthFrame = 0;
      _Spy_private->nbrChan = 0;
    }
  }
  else {
    if (_Spy_private->startedFrame) {
      _Spy_private->lengthFrame = _Spy_private->lengthFrame + length;
      _Spy_private->nbrChan++;
    }
  }
}

static void IRAM_ATTR processLevelDuration ( unsigned int level, unsigned int durationInTicks, Spy_private_area_t *_Spy_private) {
  unsigned int duration = durationInTicks / TICK_SPY_PER_US ;
  if (duration == 0 || duration > SPY_MIN_IDLE) return;
  if (level == 0 ) {
    _Spy_private->result_spy.maxLow = max (_Spy_private->result_spy.maxLow, duration);
    _Spy_private->result_spy.minLow = min (_Spy_private->result_spy.minLow, duration);
    _Spy_private->sumLow = _Spy_private->sumLow + duration;
    _Spy_private->nbrLow++;
  } else {  // level is 1
    _Spy_private->result_spy.maxHigh = max (_Spy_private->result_spy.maxHigh, duration);
    _Spy_private->result_spy.minHigh = min (_Spy_private->result_spy.minHigh, duration);
    _Spy_private->sumHigh = _Spy_private->sumHigh + duration;
    _Spy_private->nbrHigh++;
  }
}

static void  IRAM_ATTR analyzeSymbols(Spy_private_area_t *_Spy_private) {
  _Spy_private->startedFrame = false;
  _Spy_private->lengthFrame = 0;
  _Spy_private->nbrChan = 0;
  for (int i = 1; i <= _Spy_private->num_symbols; i++) {
    manageFrame(_Spy_private->raw_symbols[i - 1].duration0, _Spy_private );
    processLevelDuration(_Spy_private->raw_symbols[i - 1].level0,
                         _Spy_private->raw_symbols[i - 1].duration0, _Spy_private);
    manageFrame(_Spy_private->raw_symbols[i - 1].duration1, _Spy_private);
    processLevelDuration(_Spy_private->raw_symbols[i - 1].level1,
                         _Spy_private->raw_symbols[i - 1].duration1, _Spy_private);
  }

  if (_Spy_private->sumHigh / _Spy_private->nbrHigh / TICK_SPY_PER_US < 500 )  _Spy_private->result_spy.polarity = RISING;
  else  _Spy_private->result_spy.polarity = FALLING;
}

// Call back. Must be static to be registered and communicate with the object
static bool IRAM_ATTR check_rmt_rx_done_callback(rmt_channel_handle_t channel, const rmt_rx_done_event_data_t *edata, void *user_data)
{
  Spy_private_area_t *pt_Spy_private = (Spy_private_area_t *) user_data;
  pt_Spy_private->num_symbols = edata->num_symbols;
  analyzeSymbols(pt_Spy_private);
  if (edata->flags.is_last) {  //  current received data are the last part of the transaction ??
    // should never occurs on a "normal RC ppm stream" because we enabled partial reception feature en_partial_rx.
    //  and for SPY we have a large signal_range_max_ns: read should never stop !!
    // Return only the minChan and manChan as 0
    pt_Spy_private->result_spy.minChan = 0;
    pt_Spy_private->result_spy.maxChan = 0;
    // "anything that is not 0 is true."
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    unsigned long ii = micros();
    xQueueOverwriteFromISR(pt_Spy_private->Spy_queue, &ii, &xHigherPriorityTaskWoken);
    return (xHigherPriorityTaskWoken == pdTRUE);
  }

  // "anything that is not 0 is true."
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  if (millis() < pt_Spy_private->timeStopSpy ) return pdFALSE;
  // end of test
  ESP_ERROR_CHECK(rmt_disable(pt_Spy_private->Spy_channel));
  // "anything that is not 0 is true."
  unsigned long ii = micros();
  xQueueOverwriteFromISR(pt_Spy_private->Spy_queue, &ii, &xHigherPriorityTaskWoken);
  return (xHigherPriorityTaskWoken == pdTRUE);
}
