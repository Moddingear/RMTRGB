#pragma once

#include <stdint.h>
#include <driver/rmt_encoder.h>

class rmtrgb
{
private:
	struct rmt_led_strip_encoder{
		rmt_encoder_t base;
		rmt_encoder_t *bytes_encoder;
		rmt_encoder_t *copy_encoder;
		int state;
		rmt_symbol_word_t reset_code;
	};
	rmt_channel_handle_t led_chan = NULL;
	struct rmt_led_strip_encoder *encoder;
public:

	struct ws2812_format
	{
		uint8_t g,r,b;

		// hue (h) 0-360
		// saturation (s) 0-100
		// value (v) 0-100
		void from_hsv(uint16_t h, uint8_t s, uint8_t v)
		{
			h %= 360; // h -> [0,360]
			uint32_t rgb_max = v * 2.55f;
			uint32_t rgb_min = rgb_max * (100 - s) / 100.0f;

			uint32_t i = h / 60;
			uint32_t diff = h % 60;

			// RGB adjustment amount by hue
			uint32_t rgb_adj = (rgb_max - rgb_min) * diff / 60;

			switch (i) {
			case 0:
				r = rgb_max;
				g = rgb_min + rgb_adj;
				b = rgb_min;
				break;
			case 1:
				r = rgb_max - rgb_adj;
				g = rgb_max;
				b = rgb_min;
				break;
			case 2:
				r = rgb_min;
				g = rgb_max;
				b = rgb_min + rgb_adj;
				break;
			case 3:
				r = rgb_min;
				g = rgb_max - rgb_adj;
				b = rgb_max;
				break;
			case 4:
				r = rgb_min + rgb_adj;
				g = rgb_min;
				b = rgb_max;
				break;
			default:
				r = rgb_max;
				g = rgb_min;
				b = rgb_max - rgb_adj;
				break;
			}
		}
	};

	rmtrgb();
	rmtrgb(gpio_num_t gpio, uint32_t frequency = 20000000);
	~rmtrgb();

	void init(gpio_num_t gpio, uint32_t frequency = 20000000);

	//data usually in grb format for ws2812, length in bytes
	void send(const void* data, size_t len, bool blocking = true);

	bool is_transmitting() const;


private:
	static size_t rmt_encode_led_strip(rmt_encoder_t *encoder, rmt_channel_handle_t channel, const void *primary_data, size_t data_size, rmt_encode_state_t *ret_state);
	static esp_err_t rmt_del_led_strip_encoder(rmt_encoder_t *encoder);
	static esp_err_t rmt_led_strip_encoder_reset(rmt_encoder_t *encoder);
	static bool rmt_tx_done_callback(rmt_channel_handle_t tx_chan, const rmt_tx_done_event_data_t *edata, void *user_ctx);

	/**
	 * @brief Create RMT encoder for encoding LED strip pixels into RMT symbols
	 *
	 * @param[in] config Encoder configuration
	 * @param[out] ret_encoder Returned encoder handle
	 * @return
	 *      - ESP_ERR_INVALID_ARG for any invalid arguments
	 *      - ESP_ERR_NO_MEM out of memory when creating led strip encoder
	 *      - ESP_OK if creating encoder successfully
	 */
	esp_err_t rmt_new_led_strip_encoder(uint32_t frequency);

	bool transmission_ongoing = false;
};