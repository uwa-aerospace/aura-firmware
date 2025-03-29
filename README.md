# IMPORTANT: Change board model on platformio.ini when developing on actual flight computer
Currently using DevKitC to prep drivers and dev environment

# Git rules
We are using a gitflow workflow: [Gitflow guide here](https://www.atlassian.com/git/tutorials/comparing-workflows/gitflow-workflow)
1. Do NOT merge to `main` without approval of 2 reviewers
2. Merges to `staging` should be reviewed by 1 person
3. Notify all other developers when pushing/merging to `staging`

## Notes/fixes:
### USB Serial output not showing
Add this option to `platformio.ini`:
```
build_flags = 
	-D ARDUINO_USB_MODE=1
	-D ARDUINO_USB_CDC_ON_BOOT=1
```

### MacOS 100% CPU usage
`sudo killall python` to terminate all python processes. Happens when the program crashes or debug execution unexpectedly stops, still unsolved (been 3 years)

Issue tracker: https://github.com/platformio/platformio-vscode-ide/issues/2891

### Concurrency issue with tasks
Core 0 is significantly more congested because it runs RTOS, system interrupts and WiFi.

Expensive/blocking tasks should not be pinned to Core 0, as they can get unexpectedly interrupted. Use Core 1 instead