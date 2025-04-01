# IMPORTANT: Change board model on platformio.ini when developing on actual flight computer
Currently using DevKitC to prep drivers and dev environment

# Git rules
We are using a gitflow workflow: [Gitflow guide here](https://www.atlassian.com/git/tutorials/comparing-workflows/gitflow-workflow)
1. Do NOT merge to `main` without approval of 2 reviewers
2. Merges to `staging` should be reviewed by 1 person
3. Notify all other developers when pushing/merging to `staging`

# Installation/setup
1. Install the `PlatformIO IDE` extension on VSCode
2. Clone this repository

PlatformIO should perform the setup automatically.

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