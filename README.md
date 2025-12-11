# OpenController-ESP32

Custom controller that works as both an XInput and Nintendo Pro Controller over Bluetooth.
Uses an ESP32 as the brains, and an ICM-20948 to replace the right stick with gyro inputs.

## Building
There's vscode tasks setup that will configure all the needed requirements, as well as pass your ESP32 through so it can be flashed/monitored (as long as your on linux, your mileage may vary in other OS's).

The bare minimum steps required to build one as documented are:

1. Print [all the STL's](3d/STL/).
2. Install docker, press Control+Shift+B in vscode, and choose `Flash` while your ESP32 is plugged in.
3. Wire everything up exactly as specified in the [schematic](hardware/Controller%20Wiring.kicad_sch).

## Using the controller
* the default mode is Switch Pro
* hold down the X button during boot to use XInput instead
* getting the gyro working nice was hard, so i added the gyro_on button. Assuming you're wearing the blaster on your right arm you can press the gyro_on button against your body to make the controller process gyro events

## Tips For Customising
* Everything in the model is scaled for my size, you'll probably need to tweak positions.
* Use pins from ADC channel 1 for the joystick, unless you feel like adding more code to the (depricated) esp-idf ADC driver to support ADC channel 2.
* If you're using a pin > 32 it'll need a pull down resistor like I've shown in the schematic. I have no idea what the ideal resistance is but I had a pile of 1kΩ lying around and they worked for me.
* The gyro code needs a lot of love (or adding in a magnetomer to make the x-axis easier).
