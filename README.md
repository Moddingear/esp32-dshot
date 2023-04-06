# ESP32 ESC
Expanded DSHOT control for ESCs, with regards to the base esp32-dshot library.

In comparison with https://github.com/derdoktor667/DShotRMT, this makes you able to call all DSHOT commands, such as turning on LEDs or beeping. 

In comparison with the base library, there's more commands available, 3D mode (bidirectionnal), some bugs fixed and uninstall is implemented. Also the throttle is 0-1999 instead of 48-2047.
It's still using the RMT peripheral so performance is great.

Can be used with PlatformIO, using lib_deps.