#include "esp_check.h"
#include <rmtrgb.hpp>
#include <driver/rmt_tx.h>
#include <freertos/FreeRTOS.h>
#include <memory.h>

rmtrgb::rmtrgb()
{}

rmtrgb::rmtrgb(gpio_num_t gpio, uint32_t frequency)
{
	init(gpio, frequency);
}

rmtrgb::~rmtrgb()
{
	rmt_del_led_strip_encoder(&encoder->base);
	ESP_ERROR_CHECK(rmt_del_channel(led_chan));
}

void rmtrgb::init(gpio_num_t gpio, uint32_t frequency)
{
	rmt_tx_channel_config_t tx_chan_config = {};
	memset(&tx_chan_config, 0, sizeof(tx_chan_config));
	tx_chan_config.clk_src = RMT_CLK_SRC_DEFAULT, // select source clock
	tx_chan_config.gpio_num = gpio;
	tx_chan_config.mem_block_symbols = 512; // increasing the block size can make the LEDs flicker less
	tx_chan_config.resolution_hz = frequency;
	tx_chan_config.trans_queue_depth = 4; // set the number of transactions that can be pending in the background
	tx_chan_config.flags.with_dma = 1;
	ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_chan_config, &led_chan));

	ESP_ERROR_CHECK(rmt_new_led_strip_encoder(frequency));
	ESP_ERROR_CHECK(rmt_enable(led_chan));
	rmt_tx_event_callbacks_t callbacks = {
		.on_trans_done = &rmtrgb::rmt_tx_done_callback,
	};
	ESP_ERROR_CHECK(rmt_tx_register_event_callbacks(led_chan, &callbacks, this));
}

void rmtrgb::send(const void* data, size_t len, bool blocking)
{
	rmt_transmit_config_t tx_config;
	tx_config.loop_count = 0; // no transfer loop
	tx_config.flags.eot_level = 0;
	tx_config.flags.queue_nonblocking = !blocking;
	transmission_ongoing = true;
	auto ret = rmt_transmit(led_chan, &encoder->base, data, len, &tx_config);
	if (blocking)
	{
		ESP_ERROR_CHECK(ret);
		//printf("transmit ok \n");
		ESP_ERROR_CHECK(rmt_tx_wait_all_done(led_chan, portMAX_DELAY));
		//printf("tx_wait ok\n");
	}
}


bool rmtrgb::is_transmitting() const
{
	return transmission_ongoing;
}


size_t rmtrgb::rmt_encode_led_strip(rmt_encoder_t *encoder, rmt_channel_handle_t channel, const void *primary_data, size_t data_size, rmt_encode_state_t *ret_state)
{
	struct rmt_led_strip_encoder *led_encoder = __containerof(encoder, struct rmt_led_strip_encoder, base);
	rmt_encoder_handle_t bytes_encoder = led_encoder->bytes_encoder;
	rmt_encoder_handle_t copy_encoder = led_encoder->copy_encoder;
	rmt_encode_state_t session_state = RMT_ENCODING_RESET;
	int state = RMT_ENCODING_RESET;
	size_t encoded_symbols = 0;
	switch (led_encoder->state) {
	case 0: // send RGB data
		encoded_symbols += bytes_encoder->encode(bytes_encoder, channel, primary_data, data_size, &session_state);
		if (session_state & RMT_ENCODING_COMPLETE) {
			led_encoder->state = 1; // switch to next state when current encoding session finished
		}
		if (session_state & RMT_ENCODING_MEM_FULL) {
			state |= (int)RMT_ENCODING_MEM_FULL;
			break; // yield if there's no free space for encoding artifacts
		}
	// fall-through
	case 1: // send reset code
		encoded_symbols += copy_encoder->encode(copy_encoder, channel, &led_encoder->reset_code,
												sizeof(led_encoder->reset_code), &session_state);
		if (session_state & RMT_ENCODING_COMPLETE) {
			led_encoder->state = RMT_ENCODING_RESET; // back to the initial encoding session
			state |= (int)RMT_ENCODING_COMPLETE;
		}
		if (session_state & RMT_ENCODING_MEM_FULL) {
			state |= (int)RMT_ENCODING_MEM_FULL;
			break; // yield if there's no free space for encoding artifacts
		}
	}
	*ret_state = (rmt_encode_state_t)state;
	return encoded_symbols;
}

esp_err_t rmtrgb::rmt_del_led_strip_encoder(rmt_encoder_t *encoder)
{
	struct rmt_led_strip_encoder *led_encoder = __containerof(encoder, struct rmt_led_strip_encoder, base);
	rmt_del_encoder(led_encoder->bytes_encoder);
	rmt_del_encoder(led_encoder->copy_encoder);
	free(led_encoder);
	led_encoder = nullptr;
	return ESP_OK;
}

esp_err_t rmtrgb::rmt_led_strip_encoder_reset(rmt_encoder_t *encoder)
{
	struct rmt_led_strip_encoder *led_encoder = __containerof(encoder, struct rmt_led_strip_encoder, base);
	rmt_encoder_reset(led_encoder->bytes_encoder);
	rmt_encoder_reset(led_encoder->copy_encoder);
	led_encoder->state = RMT_ENCODING_RESET;
	return ESP_OK;
}

esp_err_t rmtrgb::rmt_new_led_strip_encoder(uint32_t frequency)
{
	encoder = (rmt_led_strip_encoder*)rmt_alloc_encoder_mem(sizeof(rmt_led_strip_encoder));
	encoder->base.encode = &rmt_encode_led_strip;
	encoder->base.del = &rmt_del_led_strip_encoder;
	encoder->base.reset = &rmt_led_strip_encoder_reset;
	// different led strip might have its own timing requirements, following parameter is for WS2812
	rmt_bytes_encoder_config_t bytes_encoder_config;
	bytes_encoder_config.bit0.level0 = 1;
	bytes_encoder_config.bit0.duration0 = 0.4 * frequency / 1000000; // T0H=0.3us
	bytes_encoder_config.bit0.level1 = 0;
	bytes_encoder_config.bit0.duration1 = 0.85 * frequency / 1000000; // T0L=0.9us
	bytes_encoder_config.bit1.level0 = 1;
	bytes_encoder_config.bit1.duration0 = 0.8 * frequency / 1000000; // T1H=0.9us
	bytes_encoder_config.bit1.level1 = 0;
	bytes_encoder_config.bit1.duration1 = 0.45 * frequency / 1000000; // T1L=0.3us
	bytes_encoder_config.flags.msb_first = 1; // WS2812 transfer bit order: G7...G0R7...R0B7...B0
	ESP_ERROR_CHECK(rmt_new_bytes_encoder(&bytes_encoder_config, &encoder->bytes_encoder));
	rmt_copy_encoder_config_t copy_encoder_config = {};
	ESP_ERROR_CHECK(rmt_new_copy_encoder(&copy_encoder_config, &encoder->copy_encoder));

	uint32_t reset_ticks = frequency / 1000000 * 50 / 2; // reset code duration defaults to 50us
	
	encoder->reset_code.level0 = 0;
	encoder->reset_code.duration0 = reset_ticks;
	encoder->reset_code.level1 = 0;
	encoder->reset_code.duration1 = reset_ticks;
	return ESP_OK;
}

bool IRAM_ATTR rmtrgb::rmt_tx_done_callback(rmt_channel_handle_t tx_chan, const rmt_tx_done_event_data_t *edata, void *user_ctx)
{
	auto controller = reinterpret_cast<rmtrgb*>(user_ctx);
	controller->transmission_ongoing = false;
	return false;
}