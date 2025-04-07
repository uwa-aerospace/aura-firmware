# Installation/setup
1. Install the `PlatformIO IDE` extension on VSCode
2. Clone this repository

## IMPORTANT NOTICE
PlatformIO does not support generic ESP32-S3 with 4MB of flash. Uploading a program with `esp32-s3-devkitc-1` default settings will result in a bootloop.

You must navigate to your `.platformio` installation folder, then go to `platforms/espressif32/boards/esp32-s3-devkitc-1.json`

Change `partitions` to `default.csv` and `flash_size` to `4MB`.

[Forum solution is here](https://community.platformio.org/t/changed-chip-not-work-please-help-me/33883).

### How to use the library:
Click on the (found on bottom bar, or top right):
- Check icon to compile
- Arrow icon to upload
- F5 to debug (or open the debug tab in the sidebar)
- Plug icon to open Serial Monitor

# Notes/fixes:
### 1. Cannot upload program to board
Due to serial error, no device detected etc.

Enter bootloader download mode by pressing BOOT0 and Reset, let go of reset, wait 1 second then let go of BOOT0 

### 2. USB Serial output not showing
Add this option to `platformio.ini`:
```
build_flags = 
	-D ARDUINO_USB_MODE=1
	-D ARDUINO_USB_CDC_ON_BOOT=1
```
(already done in this project)

### 3. Serial.println not showing properly
Use `ESP_LOGI` or `printf` for serial output. There are some issues with `Serial.print/Serial.println`

`-D CORE_DEBUG_LEVEL=5` flag needs to be set for `ESP_LOGI` to work

### 4. MacOS 100% CPU usage
`sudo killall python` to terminate all python processes. Happens when the program crashes or debug execution unexpectedly stops, still unsolved (been 3 years)

Issue tracker: https://github.com/platformio/platformio-vscode-ide/issues/2891

# Useful tips/knowledge
- `ESP_LOGE` will still print to serial even if the program is being debugged via USB.