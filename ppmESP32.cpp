#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/rmt_rx.h"
#include "ppmESP32.h"
#include <stdint.h>

// ESP_ERROR_CHECK
// ESP_ERROR_CHECK
//ppmReader:: ppmReader(void ) {
ppmReader:: ppmReader( ) {
}
void ppmReader::end() {
  if (_RX_private != NULL) {
    rmt_disable(_RX_private->rx_channel);
    rmt_del_channel(_RX_private->rx_channel);
    delete _RX_private;
    _RX_private = NULL;
  }
}
// via taskENTER_CRITICAL()/taskEXIT_CRITICAL()
// POtential race with callBack ??
unsigned long ppmReader::newFrame() {
  // "anything that is not 0 is true."
  unsigned long ii;
  if (xQueueReceive(_RX_private->RX_queue, &ii, 0)) return ii;
  return 0;
}
/*
  unsigned long ppmReader::newFrame() {
  // "anything that is not 0 is true."
  unsigned long ii;
  if (ii = _RX_private->timeFrame) {
    _RX_private->timeFrame = 0;
    return ii;
  }
  return 0;
  }
*/
int* ppmReader::begin(uint8_t rxPin, uint8_t nbr_RX_Channels)
{
  Serial.printf("Debut pin %d\n", rxPin);
  ESP_LOGI(TAG, "create RMT RX channel");
  rmt_rx_channel_config_t rx_channel_cfg = {
    .gpio_num   = static_cast<gpio_num_t> (rxPin),
    .clk_src = RMT_CLK_SRC_DEFAULT,
    .resolution_hz = PPM_RESOLUTION_HZ,
    .mem_block_symbols = 64, // amount of RMT symbols that the channel can store at a time
    // (mini 64 for ESP32, 48 for ESP32C3 or S3)
  };
  // data for communication with the callback
  _RX_private = new (RX_private_area_t );
  if (_RX_private == NULL) return NULL;
  if (nbr_RX_Channels > MAX_PPM_CHANNELS_RX) return NULL;
  _RX_private->nbr_RX_Channels = nbr_RX_Channels;
  Serial.printf("In begin. Param nbr_RX_Channels:%d, _RX_private->nbr_RX_Channels:%d\n", nbr_RX_Channels, _RX_private->nbr_RX_Channels);
  _RX_private->receive_config.signal_range_max_ns = RX_minimum_space * 1000;
  ESP_ERROR_CHECK(rmt_new_rx_channel(&rx_channel_cfg, &_RX_private->rx_channel));
  ESP_LOGI(TAG, "register RX done callback");
  rmt_rx_event_callbacks_t cbs = {
    .on_recv_done =  example_rmt_rx_done_callback,  // de type rmt_rx_done_callback_t
  };
  ESP_ERROR_CHECK(rmt_rx_register_event_callbacks(_RX_private->rx_channel, &cbs, _RX_private));
  ESP_LOGI(TAG, "enable RMT RX channels");
  ESP_ERROR_CHECK(rmt_enable(_RX_private->rx_channel));

  _RX_private->RX_queue = xQueueCreate(1, sizeof(unsigned long));
  assert( _RX_private->RX_queue);
  // start reading
  ESP_ERROR_CHECK(rmt_receive(_RX_private->rx_channel, _RX_private-> raw_symbols,
                              sizeof(_RX_private->raw_symbols), &_RX_private->receive_config)) ;
  return  _RX_private->RX_Channels_values;
}  // end of begin

// Call back. Must be static to be registered and communicate with the object
bool example_rmt_rx_done_callback(rmt_channel_handle_t channel, const rmt_rx_done_event_data_t *edata, void *user_data)
{
  int static expectedCh;
  int static cnt = 0;
  int static nbrReceived = 0;
  digitalWrite(gpioDebug, HIGH);
  RX_private_area_t *pt_RX_private = (RX_private_area_t *) user_data;
  // the last symbols contains only the end pulse of the last channel. We receive 1 symbol more than the nbr of channels
  pt_RX_private->RX_Channels_values[0] =  edata->num_symbols - 1;
  for (int i = 1 ; i < edata->num_symbols; i++) {
    // convert in us, round to the nearest integer value
    pt_RX_private->RX_Channels_values[i] = ((edata ->received_symbols[i - 1].duration1 + TICK_PER_US / 2) / TICK_PER_US)
                                           + ((edata ->received_symbols[i - 1].duration0 + TICK_PER_US / 2) / TICK_PER_US);
  }
  /*
    nbrReceived++;

    // test values are between 1000 and 1999. Each channel in a frrame have the same value. Increase by 1 at each frame
    for (int i = 1; i <= edata->num_symbols - 1 ; i++) {
    if (pt_RX_private->RX_Channels_values[i] != expectedCh ) {
      Serial.printf(" ------ err seq. Frames received:%d, Channel:%d, Value:%d, Expected:%d, (raw:%d, dur0:%d, dur1:%d++ )",
                    nbrReceived, i, pt_RX_private->RX_Channels_values[i], expectedCh,
                    edata ->received_symbols[i - 1].duration1 + edata ->received_symbols[i - 1].duration0,
                    edata ->received_symbols[i - 1].duration0 , edata ->received_symbols[i - 1].duration1);
      Serial.println();
      break;
    }
    }
    expectedCh = pt_RX_private->RX_Channels_values[1] + 1; // assuming that [1] is not too bad ...
    if  ( expectedCh > 1999) {
    Serial.printf(" +++Frames received:%d\n", nbrReceived);
    expectedCh = 1000;
    }
  */

  // Restart a read
  // esp_err_t rmt_receive(rmt_channel_handle_t rx_channel, void *buffer, size_t buffer_size, const rmt_receive_config_t *config)
  ESP_ERROR_CHECK(rmt_receive(pt_RX_private->rx_channel, pt_RX_private->raw_symbols,
                              sizeof(pt_RX_private->raw_symbols), &pt_RX_private->receive_config));
  // "anything that is not 0 is true."
  // pt_RX_private->timeFrame = micros();
  unsigned long ii = micros();
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  xQueueOverwriteFromISR(pt_RX_private->RX_queue, &ii, &xHigherPriorityTaskWoken);
  digitalWrite(gpioDebug, LOW);
  return (xHigherPriorityTaskWoken == pdTRUE);
}
