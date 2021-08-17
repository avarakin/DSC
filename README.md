# DSC


Here is another Digital Settings Circles project, this time it is not using rotary encoders.

Instead, it is using smart sensor BNO055 from Bosch:   https://www.bosch-se...sensors/bno055/

Obvious advantage of this approach is that there is no need of any mechanical modifications of the scope, you just stick the whole thing to OTA.

I was thinking about building this project for a while and bought the sensor few months ago, but did not have a chance to work on it.

Last weekend I was stuck at home due to some hopefully minor medical condition, so decided to start working on this project.

 

I've made some very good progress:

1. Built a prototype hardware piece. It is still just a wired board without any box.

2. Wrote software. It is using pieces of code from another similar project - https://www.cloudyni...for-diy-makers/

3. Have done some bench testing

 

Results of the bench testing seem to be very promising so far: SkySafary is tracking the position of the board fairly accurate. There are no jumps, the pointer in Sky Safari is moving smoothly and quickly.

The code is available here: https://github.com/avarakin/DSC

 

Some notes on the sensors:

1. The original BNO055 sensors seem to be discontinued so the prices for them went all the way up to $40, although with luck, you might be able to find them for $20

2. On the other hand, the similar sensors BNO080/085/086 are available from CEVA. I read that it is the same hardware as BNO055, but different firmware. I was able to find boards with these sensors for $20. I doubt they would work with my program though

3. I am using the sensor in a very simple mode, where position is determined based on 6 sensors - 3 accelerometers and 3 gyros. I decided not to use magnetometers for now. Surprisingly, gyros are working pretty stable and I am not seeing much of a drift

4. Here is more information on the sensor board I am using:   It also gives steps on how to make the board usable.

 

Next steps are to attach the board to some scope (most likely 10" DOB) and try it under stars.



Feel free to try and let me know if you have any questions



 

