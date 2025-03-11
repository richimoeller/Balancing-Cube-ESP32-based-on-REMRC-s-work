Balancing Cube ESP32 based on REMRC's work
------------------------------------------

There already a few cubes out there based on REMRCs design.

All the credits go to him.

I redesigned the cube in Fusion 360 since I wanted to have an unobstructed view at the reaction wheels. All other parts are following closely REMRC's design.
Basically the cube worked right from the start with no changing of the tuning parameters needed. My reaction wheels have a weight of 72g. I use a 11,1V 550mAh battery.
The code uses the encoders. You have to wire them to the ESP32(30pin). I needed to change the code a little because the pwm_channel assignment used by REMRC is deprecated meanwhile.

Just follow the calibration procedure provided by REMRC.

I also added the functionality of letting the cube turn on its vertex by sending the commands r+, r- and rs(Stop) via a bluetooth terminal. However the cube will not turn forever in one direction. The wheels will speed up and eventually saturate. Just change the direction or stop it in due time. 

https://www.youtube.com/watch?v=GABWHTa6_v8

I added motor numbers to the schematic as well as to the 3D picture. This way you should be able to connect the motors in the correct order right from the beginning. Feel free to use the Fritzing-board provided. 

It's an amazing project. Enjoy!!
