# ![OpenTally Icon](https://git.visnovsky.us/Matt/open-tally/-/raw/master/icons/Icon-02.png) Open Tally Wi-Fi Client

The Open Tally Wi-Fi Client is an accessory program that allows you to connect to a Tally Arbiter server and control an ESP32 Arduino device based on the incoming tally information.

To learn more about the Tally Arbiter project, [click here](http://github.com/josephdadams/tallyarbiter).

## Installing Sketch and Libraries
1. Follow the [tutorial on the M5Stack website](https://docs.m5stack.com/#/en/arduino/arduino_development) to download, install, and configure the Arduino IDE program. This is necessary to compile the code for your device.
1. Once you have the Arduino IDE installed and configured for your OS, install the following libraries (if not already installed):
	* `Websockets`
	* `SocketIoClient`
	* `Arduino_JSON`
	* `MultiButton`

These will have to be included with the sketch file in order for it to compile properly.
### Modification of SocketIoClient file for ESP32 compatibility
One library file will need to be modified in order to work properly: `SocketIoClient.cpp`.

Line `41` of this file reads, `hexdump(payload, length);`. This function is not declared properly and thus will error out.

The fix is to simply comment out this line as it is not needed and is only for debugging purposes. Modify the line so it now reads, `//hexdump(payload, length);`.

## Compile and Upload the Sketch to the Device
1. Once all libraries are downloaded, open the `tallyarbiter-m5stickc.ino` file in the Arduino IDE.
2. Modify these lines are the top of the file to reflect your wireless network and Tally Arbiter server settings:
	```c++
	//Wifi SSID and password
	const char * networkSSID = "YourNetwork";
	const char * networkPass = "YourPassword";

	//Tally Arbiter Server
	const char * tallyarbiter_host = "192.168.1.100";
	const int tallyarbiter_port = 4455;
	```
3. Save the file.
4. Connect your OpenTally WiFi Client device to the computer via the provided USB-C cable.
5. If not already on, power the device on by holding down the power button (located on the bottom left-hand side) for a couple seconds.
6. Go to Tools > Board > ESP32 Arduino > and choose `OpenTally WiFi Client`. If it's not listed, you may need to install it through the Boards Manager.
7. Go to Tools > Upload Speed > and choose `750000` (one less from the maximum speed).
8. Go to Tools > Port > and choose the serial port that represents your device.
9. Go to Sketch > and choose `Upload`. The code will compile and upload to the device.

Once the code is successfully compiled and uploaded to the device. the OpenTally WiFi Client will boot up and automatically try to connect to your Tally Arbiter server. It will auto-assign itself to the first Device on the server, and you can reassign it through the Settings GUI of Tally Arbiter.

## Using the Device
When you turn on the OpenTally WiFi Client device after it has been programmed, it will automatically connect to the wireless network using the settings provided, and then initiate a connection to the Tally Arbiter server. If the server is offline, just reboot the device after the server is back online.

Other features using various button combinations may be added at a later date with an upgrade to this code.

## Improvements and Suggestions
I welcome all improvements and suggestions. You can submit issues and pull requests, or contact me through GitHub.

## Credits
- The amazing Joseph Adams and his hard work on the TallyArbiter project.
- Thank you to [Guido Visser](https://github.com/guido-visser), inspiration for this listener client came from his project, [vMix OpenTally WiFi Client Tally Light](https://github.com/guido-visser/vMix-M5Stick-Tally-Light).
