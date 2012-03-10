The JTAG Whisperer
==================

With this library, your Arduino is an XSVF player. Use it to program CPLDs and FPGAs!

This code is rewriten from Arduino/C++ language to C. Now, it do not require Arduino IDE nor
Arduino itself. You can compile this project by `make` and programme it into any ATMega328P with
Arduino bootloader. 

For UART communication, the [AVR UART library](http://beaststwo.org/avr-uart/index.shtml) is used.

Although it is still not finished, it should be working.

Warning
=======

**Most Arduinos are 5-volt devices. Many JTAG devices are not 5-volt devices and will die if they get anywhere near 5 volts. Be sure your device is 5-volt tolerant or that you use an appropriate buffer for the I/O lines!**

Instructions
============

You'll need Python on your desktop, as well as the Python library pyserial (try `easy_install pyserial` or `pip install pyserial` to get it). You'll also need a target JTAG-speaking board of some kind, an Arduino, and four jumpers to connect four of the Arduino pins to your target board's JTAG headers.

1. Find your Arduino IDE's sketches directory. This is the directory where you keep your own sketches.

1. If there isn't already one, create a `libraries` directory inside the sketches directory.

1. Unzip the project archive into `libraries`. It should end up looking like this: `[Your Arduino sketches directory]/libraries/JTAGWhisperer/JTAGWhisperer.h (etc.)`

1. Restart your Arduino IDE and you should see JTAGWhisperer among the libraries in the Sketch -> Import Library... menu.

1. Open the example JTAGWhisperer sketch. Upload it to your Arduino.

1. Get a Xilinx XC9572XL CPLD. Hook up your CPLD's JTAG pins to the Arduino: pins 8, 9, 10, 11 to TMS, TDI, TDO, and TCK, respectively. (If you have TCK on pin 11, you probably did it right.) On an Arduino MEGA or MEGA2560, use pins 53, 52, 51, 50 accordingly. **Re-read the warning above about I/O voltage. You just hooked up 5-volt I/O to your device, and that might let out the magic smoke!**

1. Apply power to the CPLD (note that the XC9500XL series are 3.3-volt devices!). Do not forget to set the '9572 VIO to 3.3V. Try asking it its device ID as follows, from the root of the source directory:

`./send_xsvf -p /dev/tty.your_arduino_serial_port xsvf/XC9572XL/DeviceID.xsvf`

1. You should see a back-and-forth that ends up with a "Success" message. If you rebuild the sketch with the DEBUG messages enabled in SerialComm.h, you'll see the actual device ID that's reported.

1. Now you're ready to write data to your CPLD. Execute either of these:

`./send_xsvf -p /dev/tty.your_arduino_serial_port xsvf/XC9572XL/ProgramButtonLED.xsvf`

`./send_xsvf -p /dev/tty.your_arduino_serial_port xsvf/XC9572XL/AlternateLEDsOnButton.xsvf`

The first one blinks D1 when you press the button. The second one alternates between D1 and D2 when you hold down the button.

Generate your own XSVF files using Xilinx iMPACT, or download them from the web. Just make sure they're for your device.

Credits
=======

By [Mike Tsao](http://www.sowbug.com/). Project on [GitHub](https://github.com/sowbug/JTAGWhisperer/).

Rewriten and modified by [Vlastimil Slintak](http://uart.cz) ([GitHub](https://github.com/slintak/JTAGWhisperer)).

Many thanks to [Ben](https://github.com/ben0109/XSVF-Player/), who adapted the Xilinx XAPP058 code into something comprehensible. This code evolved from his project, and all the clever parts are his.

The guys over at [Dangerous Prototypes](http://dangerousprototypes.com/) sell an XC9572XL breakout board. It's cheap, it's open-source hardware, and their Bus Pirate tool will also program CPLDs. The "AlternateLEDsOnButton" XSVF is taken directly from their CPLD tutorials. DP are the guys who got me into CPLDs in the first place. Check them out and buy their stuff!
