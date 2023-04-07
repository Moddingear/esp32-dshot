#pragma once

#include "driver/rmt.h"

// from https://github.com/bitdump/BLHeli/blob/master/BLHeli_32%20ARM/BLHeli_32%20Firmware%20specs/Digital_Cmd_Spec.txt
enum class DSHOT_CMD : uint16_t
{
	MOTOR_STOP,						// Currently not implemented
	BEEP1,							// Wait at least length of beep (260ms) before next command
	BEEP2,							// Wait at least length of beep (260ms) before next command
	BEEP3,							// Wait at least length of beep (280ms) before next command
	BEEP4,							// Wait at least length of beep (280ms) before next command
	BEEP5,							// Wait at least length of beep (1020ms) before next command
	ESC_INFO,						// Wait at least 12ms before next command
	SPIN_DIRECTION_1,				// Need 6x, no wait required
	SPIN_DIRECTION_2,				// Need 6x, no wait required
	MODE_3D_OFF,					// Need 6x, no wait required
	MODE_3D_ON,						// Need 6x, no wait required
	SETTINGS_REQUEST,				// Currently not implemented
	SAVE_SETTINGS,					// Need 6x, wait at least 35ms before next command
	SPIN_DIRECTION_NORMAL = 20,		// Need 6x, no wait required
	SPIN_DIRECTION_REVERSED,		// Need 6x, no wait required
	LED0_ON,						// No wait required
	LED1_ON,						// No wait required
	LED2_ON,						// No wait required
	LED3_ON,						// No wait required
	LED0_OFF,						// No wait required
	LED1_OFF,						// No wait required
	LED2_OFF,						// No wait required
	LED3_OFF,						// No wait required
};

class DShotESC
{
public:
	DShotESC();
	~DShotESC();

	esp_err_t install(gpio_num_t gpio, rmt_channel_t rmtChannel, unsigned long DSHOT_FREQUENCY = 600000UL, uint8_t rmtdivider = 3);
	esp_err_t uninstall();

	//Resets the ESC by sending zeros
	//To be fair, I don't know if this does anything, but it probably doesn't hurt
	esp_err_t init();

	esp_err_t sendMotorStop();

	//Arm : BLHeli-style ESCs
	//Sends a throttle 0 commands
	esp_err_t throttleArm(int ArmDuration=400);
	//Arm : Bluejay ESC
	//Sends MOTOR_STOP for at least 300ms
	esp_err_t blueJayArm(int ArmDuration=400);
	//Send a throttle command : between 0(inclusive) and 2000(exclusive). Values above 2000 will be clamped to 1999
	esp_err_t sendThrottle(uint16_t throttle);
	//Send a 3D throttle command : between -999 and 999
	esp_err_t sendThrottle3D(int16_t throttle);
	//Invert ESC direction
	esp_err_t setReversed(bool reversed);
	//Set bidirectionnal mode
	esp_err_t set3DMode(bool active);
	//Do a beep. Tone is between 0 and 4, inclusive
	esp_err_t beep(uint8_t tone = 0);
	//Turn on or off a LED. There's support for up to 4 LEDs
	esp_err_t setLED(uint16_t led, bool active);
	//Save the settings
	esp_err_t saveSettings();

	esp_err_t WaitTXDone(TickType_t waitTime);

	uint8_t GetDivider();

	//it's protected so in case you want to call these functions, make a child class and you'll have access (at your own risk tho)
protected:
	struct dshot_packet_t
	{
		uint16_t payload;
		bool telemetry;
	};

	void setData(uint16_t data);
	static uint8_t checksum(uint16_t data);
	esp_err_t writeData(uint16_t data, bool wait);
	esp_err_t writePacket(dshot_packet_t packet, bool wait);
	esp_err_t repeatPacket(dshot_packet_t packet, int n);
	esp_err_t repeatPacketTicks(dshot_packet_t packet, TickType_t ticks);


	rmt_item32_t _dshotCmd[17];
	rmt_channel_t _rmtChannel;

	uint16_t dt_t0h, dt_t0l, //ticks to stay low and high for a 0
	dt_t1h, dt_t1l, //ticks to stay low and high for a one
	dt_tpb, //total duration of a bit (ticks per bit)
	dt_pause;
	uint8_t divider;
};
