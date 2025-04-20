#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/rmt_tx.h"
#include "ppmESP32.h"

ppmWriter:: ppmWriter(int polarity) {
  _polarity = polarity;
}
void ppmWriter::end() {
  if (_TX_private != NULL) {
    rmt_disable(_TX_private->tx_channel);
    rmt_del_channel(_TX_private->tx_channel);
    delete _TX_private;
    _TX_private = NULL;
  }
}

unsigned long ppmWriter::sentFrame() {
  // "anything that is not 0 is true."
  unsigned long ii;
  if (xQueueReceive(_TX_private->TX_queue, &ii, 0)) return ii;
  return 0;
}

int* ppmWriter::begin(uint8_t txPin, uint8_t nbr_TX_Channels)
{
  Serial.printf("Debut begin.TX pin %d\n", txPin);
  // allocate a private area for the object that can be chared with the callback
  _TX_private = new (TX_private_area_t);
  if (_TX_private == NULL) return NULL;
  // initialization of all values in the private area !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  _TX_private-> TX_minimum_frame = TX_minimum_frame * TICK_PER_US; //TX_MINIMUM_FRAME in RMT ticks
  _TX_private-> TX_minimum_space = TX_minimum_space * TICK_PER_US; //TX_MINIMUM_SPACE (end of frame)in ticks
  _TX_private-> TX_pulse_width = TX_pulse_width * TICK_PER_US; //TX_PULSE_WIDTH   start pulse (RISING or FALLING for a channel)
  _TX_private-> polarity = _polarity;
  if (nbr_TX_Channels > MAX_PPM_CHANNELS_TX) return NULL;
  _TX_private->nbr_TX_Channels = nbr_TX_Channels;
  Serial.printf("TX_minimum_frame:%d, TX_minimum_space:%d, TX_pulse_width:%d,  nbr_TX_Channels:%d\n",
                _TX_private->TX_minimum_frame, _TX_private->TX_minimum_space, _TX_private->TX_pulse_width,
                _TX_private->nbr_TX_Channels);

  _TX_private->tx_config.loop_count = 0;
  if (_polarity == RISING ) _TX_private->tx_config.flags.eot_level = 0;
  else
    _TX_private->tx_config.flags.eot_level = 1;

  // Default channels values in us
  _TX_private->TX_Channels_values[0] = nbr_TX_Channels; // not used in fact. Real channel1 is in [1], channel2 in [2]
  for (int i = 1; i <= nbr_TX_Channels ; i++) _TX_private->TX_Channels_values[i] = 1500;
  ESP_LOGI(TAG, "Create RMT TX channel");
  rmt_tx_channel_config_t tx_chan_config = {
    .gpio_num = static_cast<gpio_num_t> (txPin),
    .clk_src = RMT_CLK_SRC_DEFAULT, // select a clock that can provide needed resolution
    .resolution_hz = PPM_RESOLUTION_HZ,
    .mem_block_symbols = 64,   // 64 mini for ESP32, 48 for ESP32C3 or S3
    .trans_queue_depth = 10, // set the number of transactions that can be pending in the background
  };
  ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_chan_config, &_TX_private->tx_channel));

  ESP_LOGI(TAG, "Install ppm encoder");
  rmt_simple_encoder_config_t simple_encoder_config = {
    .callback = ppm_encode ,
    .arg = _TX_private,
    .min_chunk_size = 48 ,   // on doit pouvoir prendre moins pour une trame ppm (9 ou 10 ok ??)
  };
  ESP_ERROR_CHECK(rmt_new_simple_encoder(&simple_encoder_config, &_TX_private->ppm_encoder));

  //  esp_err_t rmt_tx_register_event_callbacks(rmt_channel_handle_t tx_channel,
  //  const rmt_tx_event_callbacks_t *cbs, void *user_data)
  rmt_tx_event_callbacks_t cbs = {
    .on_trans_done = ppmEventCallBack
  };
  ESP_ERROR_CHECK(rmt_tx_register_event_callbacks(_TX_private->tx_channel, &cbs, _TX_private));
  
  _TX_private->TX_queue = xQueueCreate(1, sizeof(unsigned long));
  assert( _TX_private->TX_queue);
  return _TX_private->TX_Channels_values;
};

void ppmWriter::start(void)
{
  ESP_LOGI(TAG, "Enable RMT TX channel");
  ESP_ERROR_CHECK(rmt_enable(_TX_private->tx_channel));
  _TX_private->ppmGoOn = true;
  // start TX
  ESP_ERROR_CHECK(rmt_transmit(_TX_private->tx_channel, _TX_private->ppm_encoder,
                               &_TX_private->TX_Channels_values[1], _TX_private->nbr_TX_Channels, &_TX_private->tx_config));
}

void ppmWriter::startTest(void) {
  _TX_private->testMode = true;
  _TX_private->cnt = 0;
  start();
}
void ppmWriter::stop(void) {
  _TX_private->ppmGoOn = false;
  _TX_private->testMode = false;

}

size_t ppm_encode(const void *data, size_t data_size, size_t symbols_written,
                  size_t symbols_free, rmt_symbol_word_t *symbols, bool *done, void *arg) {

  // digitalWrite(gpioDebug, !digitalRead(gpioDebug));
  // attention: le ESP32C3 a des blocks de 48 mots: il ne faudrait pas generer plus de symboles ..
  //    Surement OK, car 8 mot pour les 7 canaux, 1 mot pulse de fin  et qqmot pour bourage fin de trame
  // Pour ESP32:  64 mots par blocs. 48 pour ESPC2 ou S3
  int* pData = (int*)data;
  TX_private_area_t *pt_TX_private = (TX_private_area_t *) arg;
  size_t encoded_symbols = 0;
  int remaining = pt_TX_private->TX_minimum_frame;  // in ticks
  // duration sur 15bits = 32767 au max.
  // 32767 + 3000 = 35867 tick pour un canal, soit 3.5ms: pas de problème donc.
  //  for (int i = 0; i < nbr_TX_Channels; i++)
  for (int i = 0; i < data_size; i++)
  {
    //int j = i + symbols_written;
    int j = i;
    symbols[j].level0 = (pt_TX_private->polarity == RISING ) ? 1 : 0;
    symbols[j].duration0 = pt_TX_private->TX_pulse_width ;
    symbols[j].level1 = (pt_TX_private->polarity == RISING ) ? 0 : 1;
    symbols[j].duration1 = pData[i] * TICK_PER_US - pt_TX_private->TX_pulse_width;
    remaining = remaining - pData[i] * TICK_PER_US ;
    //  Serial.printf("remaining:%d\n", remaining);
    //  Serial.println(pData[i]);
    encoded_symbols += 1;
  }
  //  Serial.printf("Rem after chann:%d, encoded:%d\n", remaining, encoded_symbols);
  // stop pulse for last channel, and then we keep the level until the end of the frame
  // (chunk of 32767 ticks maximum)
  symbols[encoded_symbols ].level0 = (pt_TX_private->polarity == RISING ) ? 1 : 0;
  symbols[encoded_symbols ].duration0 = pt_TX_private->TX_pulse_width ;
  symbols[encoded_symbols ].level1 = (pt_TX_private->polarity == RISING ) ? 0 : 1;
  remaining = remaining - pt_TX_private->TX_pulse_width ;
  // extend the frame to be sure that the minimum space between frames is OK
  if (remaining < pt_TX_private-> TX_minimum_space) remaining = pt_TX_private->TX_minimum_space;
  symbols[encoded_symbols ].duration1 = min(32767, remaining);
  remaining = remaining - symbols[encoded_symbols ].duration1 ;
  encoded_symbols += 1;
  while (remaining > 0) {
    symbols[encoded_symbols ].level0 = (pt_TX_private->polarity == RISING ) ? 0 : 1;
    symbols[encoded_symbols ].duration0 = min(32767, remaining);
    remaining = remaining - symbols[encoded_symbols ].duration0 ;
    symbols[encoded_symbols ].level1 = symbols[encoded_symbols ].level0 ;
    symbols[encoded_symbols ].duration1 = 0;
    if (remaining <= 0) {
      encoded_symbols += 1;
      break;
    }
    symbols[encoded_symbols ].duration1 = min(32767, remaining);
    remaining = remaining - symbols[encoded_symbols ].duration1 ;
    encoded_symbols += 1;
  }
  *done = true;
  /*
    Serial.printf("remaining:%d, encoded_symbols:%d, symbols: ", remaining, encoded_symbols);
    for (int i = 0; i < encoded_symbols; i++) {
    Serial.printf(";%d %d %d %d ", symbols[i ].level0, symbols[i ].duration0,
                  symbols[i ].level1, symbols[i ].duration1);
    }
    Serial.println();
  */
  //digitalWrite(gpioDebug, !digitalRead(gpioDebug));
  unsigned long ii = micros();
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  xQueueOverwriteFromISR(pt_TX_private->TX_queue, &ii, &xHigherPriorityTaskWoken);
  return encoded_symbols;
}


// typedef bool (*rmt_tx_done_callback_t)(rmt_channel_handle_t tx_chan,
//  const rmt_tx_done_event_data_t*edata, void *user_ctx )
bool ppmEventCallBack(rmt_channel_handle_t tx_chan, const rmt_tx_done_event_data_t *edata, void *user_ctx )
{
  TX_private_area_t *pt_TX_private = (TX_private_area_t *) user_ctx;
  if (pt_TX_private->ppmGoOn) {
    if ( pt_TX_private->testMode) {
      pt_TX_private->cnt = (pt_TX_private->cnt + 1) % 1000;
      //    TX_Channels_values[0] just used for the channel count
      for (int i = 1; i <= pt_TX_private->nbr_TX_Channels; i++)  pt_TX_private->TX_Channels_values[i] = 1000 + pt_TX_private->cnt;
    }
    ESP_ERROR_CHECK(rmt_transmit(tx_chan, pt_TX_private->ppm_encoder, &pt_TX_private->TX_Channels_values[1], pt_TX_private->nbr_TX_Channels, &pt_TX_private->tx_config));
  }
  else
    ESP_ERROR_CHECK(rmt_disable(tx_chan));
  return true;
}
