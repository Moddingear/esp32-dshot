#include "DShotESC.h"
#include "freertos/task.h"

#define LOG_LOCAL_LEVEL ESP_LOG_DEBUG
#include "esp_log.h"

static const char *TAG = "dshot-rmt";

#define DSHOT_ERROR_CHECK(x) ({        \
	esp_err_t __ret = x;               \
	if (__ret != ESP_OK)               \
		return __ret;                  \
	__ret;                             \
})



// DSHOT Timings
#define DSHOT_TICKS_PER_BIT 19

#define DSHOT_T0H 7
#define DSHOT_T0L (DSHOT_TICKS_PER_BIT - DSHOT_T0H)

#define DSHOT_T1H 14
#define DSHOT_T1L (DSHOT_TICKS_PER_BIT - DSHOT_T1H)

#define DSHOT_PAUSE (DSHOT_TICKS_PER_BIT * 200)
// !DSHOT Timings

#define RMT_CMD_SIZE (sizeof(_dshotCmd) / sizeof(_dshotCmd[0]))

#define DSHOT_THROTTLE_MIN 48
#define DSHOT_THROTTLE_MAX 2047

#define DSHOT_ARM_DELAY (5000 / portTICK_PERIOD_MS)

DShotESC::DShotESC()
{
	_rmtChannel = RMT_CHANNEL_MAX;
	// initialize cmd buffer
	setData(0);

	// DShot packet delay + RMT end marker
	_dshotCmd[16].duration0 = DSHOT_PAUSE;
	_dshotCmd[16].level0 = 0;
	_dshotCmd[16].duration1 = 0;
	_dshotCmd[16].level1 = 0;
}

DShotESC::~DShotESC()
{
	if (_rmtChannel != RMT_CHANNEL_MAX)
	{
		uninstall();
	}
}

esp_err_t DShotESC::install(gpio_num_t gpio, rmt_channel_t rmtChannel)
{
	_rmtChannel = rmtChannel;

	rmt_config_t config;

	config.channel = rmtChannel;
	config.rmt_mode = RMT_MODE_TX;
	config.gpio_num = gpio;
	config.mem_block_num = 1;
	config.clk_div = 7;

	config.tx_config.loop_en = false;
	config.tx_config.carrier_en = false;
	config.tx_config.idle_level = RMT_IDLE_LEVEL_LOW;
	config.tx_config.idle_output_en = true;

	DSHOT_ERROR_CHECK(rmt_config(&config));

	return rmt_driver_install(rmtChannel, 0, 0);
}

esp_err_t DShotESC::uninstall()
{
	auto status = rmt_driver_uninstall(_rmtChannel);
	_rmtChannel = RMT_CHANNEL_MAX;
	return status;
}

esp_err_t DShotESC::init(bool wait)
{
	ESP_LOGD(TAG, "Sending reset command");
	for (int i = 0; i < 50; i++)
	{
		writeData(0, true);
	}

	ESP_LOGD(TAG, "Sending idle throttle");
	if (wait)
		DSHOT_ERROR_CHECK(repeatPacketTicks({DSHOT_THROTTLE_MIN, 0}, DSHOT_ARM_DELAY));
	else
		writePacket({DSHOT_THROTTLE_MIN, 0}, false);

	ESP_LOGD(TAG, "ESC armed");
	return ESP_OK;
}

esp_err_t DShotESC::sendThrottle(uint16_t throttle)
{
	if (throttle > DSHOT_THROTTLE_MAX - DSHOT_THROTTLE_MIN)
	{
		throttle = DSHOT_THROTTLE_MAX;
	}
	else
	{
		throttle += DSHOT_THROTTLE_MIN;
	}

	return writePacket({throttle, 0}, false);
}

esp_err_t DShotESC::sendThrottle3D(int16_t throttle)
{
	if (throttle > 999)
	{
		throttle = 999;
	}
	if (throttle < -999)
	{
		throttle = -999;
	}
	if (throttle < 0)
	{
		throttle = 1000-throttle;
	}
	return sendThrottle(throttle);
}

esp_err_t DShotESC::setReversed(bool reversed)
{
	DSHOT_ERROR_CHECK(repeatPacket(
			{reversed ? (uint16_t)DSHOT_CMD::SPIN_DIRECTION_REVERSED : (uint16_t)DSHOT_CMD::SPIN_DIRECTION_NORMAL, 1},
			10));
	return ESP_OK;
}

esp_err_t DShotESC::set3DMode(bool active)
{
	DSHOT_ERROR_CHECK(repeatPacket(
			{active ? (uint16_t)DSHOT_CMD::MODE_3D_ON : (uint16_t)DSHOT_CMD::MODE_3D_OFF, 1},
			10));
	return ESP_OK;
}

esp_err_t DShotESC::beep(uint8_t tone)
{
	tone += (uint16_t)DSHOT_CMD::BEEP1;
	if (tone > (uint16_t)DSHOT_CMD::BEEP5)
	{
		tone = (uint16_t)DSHOT_CMD::BEEP5;
	}
	DSHOT_ERROR_CHECK(writePacket({tone, 1}, true));
	if (tone == (uint16_t)DSHOT_CMD::BEEP5)
	{
		vTaskDelay(1020 / portTICK_PERIOD_MS);
	}
	else
	{
		vTaskDelay(260 / portTICK_PERIOD_MS);
	}
	return ESP_OK;
}

esp_err_t DShotESC::setLED(uint16_t led, bool active)
{
	led += (uint16_t)DSHOT_CMD::LED0_ON;
	if (led > (uint16_t)DSHOT_CMD::LED3_ON)
	{
		led = (uint16_t)DSHOT_CMD::LED3_ON;
	}
	if (!active)
	{
		led += (uint16_t)DSHOT_CMD::LED0_OFF - (uint16_t)DSHOT_CMD::LED0_ON;
	}
	DSHOT_ERROR_CHECK(writePacket({led, 1}, true));
	return ESP_OK;
}

esp_err_t DShotESC::saveSettings()
{
	DSHOT_ERROR_CHECK(repeatPacket(
			{(uint16_t)DSHOT_CMD::SAVE_SETTINGS, 1},
			10));
	return ESP_OK;
}

void DShotESC::setData(uint16_t data)
{
	for (int i = 0; i < 16; i++, data <<= 1)
	{
		if (data & 0x8000)
		{
			// set one
			_dshotCmd[i].duration0 = DSHOT_T1H;
			_dshotCmd[i].level0 = 1;
			_dshotCmd[i].duration1 = DSHOT_T1L;
			_dshotCmd[i].level1 = 0;
		}
		else
		{
			// set zero
			_dshotCmd[i].duration0 = DSHOT_T0H;
			_dshotCmd[i].level0 = 1;
			_dshotCmd[i].duration1 = DSHOT_T0L;
			_dshotCmd[i].level1 = 0;
		}
	}
}

uint8_t DShotESC::checksum(uint16_t data)
{
	uint16_t csum = 0;

	for (int i = 0; i < 3; i++)
	{
		csum ^= data;
		data >>= 4;
	}

	return csum & 0xf;
}

esp_err_t DShotESC::writeData(uint16_t data, bool wait)
{
	DSHOT_ERROR_CHECK(rmt_wait_tx_done(_rmtChannel, 0));

	setData(data);

	return rmt_write_items(_rmtChannel,
						   _dshotCmd, RMT_CMD_SIZE,
						   wait);
}

esp_err_t DShotESC::writePacket(dshot_packet_t packet, bool wait)
{
	uint16_t data = packet.payload;

	data <<= 1;
	data |= packet.telemetry;

	data = (data << 4) | checksum(data);

	return writeData(data, wait);
}

esp_err_t DShotESC::repeatPacket(dshot_packet_t packet, int n)
{
	for (int i = 0; i < n; i++)
	{
		DSHOT_ERROR_CHECK(writePacket(packet, true));
		portYIELD();
	}
	return ESP_OK;
}

esp_err_t DShotESC::repeatPacketTicks(dshot_packet_t packet, TickType_t ticks)
{
	DSHOT_ERROR_CHECK(rmt_wait_tx_done(_rmtChannel, ticks));

	TickType_t repeatStop = xTaskGetTickCount() + ticks;
	while (xTaskGetTickCount() < repeatStop)
	{
		DSHOT_ERROR_CHECK(writePacket(packet, false));
		vTaskDelay(1);
	}
	return ESP_OK;
}
