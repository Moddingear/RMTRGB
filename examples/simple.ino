#include <Arduino.h>

#include <rmtrgb.hpp>


rmtrgb::ws2812_format leds_data[8];
rmtrgb controller;

static const int num_leds = sizeof(leds_data)/sizeof(leds_data[0]);

void setup()
{
	//hook up the peripheral to the gpio
	controller.init((gpio_num_t)2);
}

void loop()
{
	for (size_t i = 0; i < num_leds; i++)
	{
		leds_data[i].from_hsv(((millis()/4)%360) + 360*i/num_leds, 100, 100);
	}
	controller.send(leds_data, sizeof(leds_data));
}