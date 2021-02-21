I started by looking at the following references:
https://learn.sparkfun.com/tutorials/tfmini---micro-lidar-module-hookup-guide/all
https://www.dfrobot.com/blog-1016.html
https://www.instructables.com/Benewake-LiDAR-TFmini-Complete-Guide/

Instead of the Arduino Serial library used in the example code, I modified the example to use WiringPi which comes
standard on the RPi. For references I also included in the repo in a zip file, which I had downloaded from the Sparkfun link.
http://wiringpi.com/reference/serial-library/

After messing with it a while, I had to discover that my 40-pin cable was bad, so once I wired everything up directly I was able to get readings.
The readings were constant, though, until I realized that the RPi was buffering TONS of readings unnecessarily.

I brought in the flush function and the delay function into the TFMini.cpp/.h driver code through the port layer to try and maintain usability on the Arduino if someone grabbed my version.
I also implemented the disable/enable measurement commands, a printFirmwareVersion command, and setting the Framerate. If you set it to 0, the code will trigger the reading. This sometimes locked up, though, so I'd recommend using something non-zero, it works fine with the flush implemented.
Arduino users WILL have to modify this version, as I have not tested that compatibility.