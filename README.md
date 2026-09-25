# 🛞 ***ROTATING TARGETING ARM PROJECT*** 🦾

## 📌 Overview
### What is it?
This is a motorized arm that rotates into any angle inputted into the serial monitor. Using a proportional-integral-derivative (PID) feedback loop and a gravity compensating feedforward loop, you can adjust each one's constants for further experimentation and actual usage on different setups.

### Why did I make this?
I wanted to make two projects that culminated my summer learning and self-teaching of embedded topics, and this is one of them. 

The motivation behind doing this learning in the first place was twofold: I wanted to gain relevant skills necessary to join controls subgroups in robotics teams and do research, while also pursuing an interest towards hobbyist electronics and simply creating a foundation for being able to build whatever comes to my mind in the future.

## 🧠 Applied Skills & Concepts
### Hardware:
- I2C communication protocol
- Non-blocking timers
- Finite state machines
- Custom sensor calibration (with the AS5600 angle readings)
### Controls:
- PID (Proportional Integral Derivative) loops
- Gravity compensation
- EMA (Exponential Moving Average) low-pass filter

## 🛠️ The Setup + Demo GIFs

The default variables in the code work with my specific setup, being a metal straw drilled and glued into my TT motor's wheel, with two weighted screws attached to its other end with masking tape.
<img width="4032" height="3024" alt="image1" src="https://github.com/user-attachments/assets/dd2e58a5-7c1d-4298-9912-bd1f963aac40" />

My code defines the directions with these angles:

0° = South

90° = West

180° = North

270° = East

Basically, if the arm is facing due south, it is at 0°. Due west, at 90°, and so on.


### Test 0: Live target changing
To bypass the 20 seconds of upload time I had to go through whenever changing any variable for testing, I implemented a function where you can type in the serial monitor a change instead.
So in this example, the arm's target is at 225°, but is commanded to go to 90°.

(Look for the "Set t = 90.00" in there)
<img width="1368" height="160" alt="Recording 2026-09-20 201604" src="https://github.com/user-attachments/assets/ee9aa7ac-152f-462c-9b46-5f919f768a2b" />
<sub>The 2 GIFs are slightly out of sync, but they still represent the same concept.</sub>

<img width="800" height="450" alt="ezgif-2d895ed4d655d5b4" src="https://github.com/user-attachments/assets/9835b1fb-3f7d-4322-b363-36e66eb68b2c" />

### Test 1: Going from 0° to 180° degrees
Accounts for gravity using feedforward, where the motor gets more power near 90°/270° but less near 0°/180°. 

Then gets closer to its target by seeing how far its current location is and then using its difference along with integral and derivative math to give the motors a specific power.
<img width="800" height="450" alt="ezgif-27d2c88ff16a8fe3" src="https://github.com/user-attachments/assets/1bd9b20c-8ce8-40b1-b131-c592181203c4" />

### Test 2: Going from 0° to 90° degrees
This one is a bit trickier. PID slows down the speed as the arm gets closer to the target, but the weight of the attached screws make it unfavorable for it to move any further due to a larger torque force.

So, the arm stops at around 60° degrees through. The integral part of PID, Ki, accumulates as the ~30° error is still quite prevalent. Eventually, the speed caps at abs(255), the max value that can be output through Pulse Width Modulation (PWM). But, due to there being no more momentum (it stopped at ~60° as Ki was building up), it cannot go any higher.

So, the program detects that and enters a momentum gaining state, where it drives full power into the opposite direction towards 180° through a proportional-derivative loop (PD). It then resumes the actual PID loop after overcoming said torque force in the opposite direction, which it can do due to there being more momentum!
<img width="800" height="450" alt="ezgif-262055f72d04ae00" src="https://github.com/user-attachments/assets/317233f2-1cb3-475e-a50c-3e869d5b783b" />

### Test 3: Going from 270° to 135°
Like the last test, lack of momentum means even at full power the arm can't go up that direction, so it goes the other way.

Though in this case, it overshoots and then goes slightly below the target to compensate, which causes another state change into going the opposite direction so that it can finally reach its target.
<img width="800" height="450" alt="ezgif-22567fa970a3a1bb" src="https://github.com/user-attachments/assets/c0327a42-6a85-4489-ae3a-dec5adb745b6" />

### Test 4: Applying force in both directions at 90°
If we push the arm in either direction, the program catches the error difference and corrects itself while also accounting for if it needs momentum or not due to higher torque forces.

<sub>Sorry for the lower quality GIF, the length was quite long!</sub>

<img width="800" height="450" alt="ezgif-22c1d72a87e17c88" src="https://github.com/user-attachments/assets/d1b0ed1f-6898-48a2-88cb-f6dc8961666f" />

### Test 5: Full 360° Journey
Arm goes from 0° -> 45° -> 90° -> 135° -> 180° -> 225° -> 270° -> 315° -> 360°.
Depending on the angle it was left in and where the target is, it goes to the swing up state when necessary to reach its destination.

<sub>This test also demonstrates some of the project's limitations in getting a precise angle and the timing it takes to get there. Those are written in full in their own section below.</sub>

[![Watch the video](https://youtu.be/Y7r584ksBy4)](https://youtu.be/Y7r584ksBy4)


## 🪫 Hardware used
Microcontroller: ESP32-S3

Modules: AS5600 magnetic encoder, DRV8833 motor driver

## 🗺️ The Journey!
In this section, I will list how I got into actually creating and finishing this, listing obstacles and how I overcame them.

### Learning what to make
This project first started as building a custom art stylus for digital drawing. 
I wanted to combine topics I would learn in ECE with a hobby of mine, and that is what I thought of.

However, I then tried adjusting it to incorporate my target robotics club's controls subgroup skills (emphasizing setpoint control, gravity compensation, etc.), and then realized that the stylus could not demonstrate these.
I simply could not figure features to map the stylus's functions into that related to those concepts, so I shelved that project idea for later.

So I scoped a different and smaller project to demonstrate these skills, and this is what I came up with. Being flexible here helped a lot looking in hindsight.

On the learning itself, I used Random Nerd Tutorials, an online educational blog and resource hub that taught about microcontrollers, communication protocols, electronics, etc. Other educational YouTube videos helped here too.
After spending a few weeks on learning, I moved unto building.

### Physical Structure & Angling

This part was a bit challenging. 

In this project, I use a TT Motor having the arm and an AS5600 magnetic encoder to read the motor's angle. The magnetic encoder comes with a tiny magnet piece (top right) that must be aligned a few millimeters on top of the module's chip so it can read the angle.

<img width="552" height="552" alt="image" src="https://github.com/user-attachments/assets/9a833152-354a-4391-a8e4-4d396410476f" />

This means that the motor would have to have the magnet attached while being a few millimeters away. Luckily, the motor had the same shaft on both ends (meaning both sides rotate at once), and after a few hours I thought of the design below:

<img width="1070" height="923" alt="image" src="https://github.com/user-attachments/assets/846063a3-7247-42bb-91ca-9cf81d978b16" />

And it worked from there. However, then came a new problem with the tiny magnet piece being slightly misaligned on the TT motor shaft. I taped the magnet down instead of gluing to avoid any irreversible outcomes, and in doing so it probably led to a slightly angled magnet slightly messing up the readings.
So for example, I would have the arm be pointing due west, but with the reading saying 40°.

I did not know how to fix this, so I got help from an AI to identify and troubleshoot the problem, and it came up with a dual-harmonic sinusoidal correction. It's kind of akin to noise canceling where opposing waves are added to then cancel out, but this time for the angle reading. But it broke in between the 0°-90° and 180°-270° degree ranges, and at this point I was dealing with complex problems I had not learned before.

So I scratched that. Eventually after more testing, I landed on a linear interpolation method instead, where linear equations bridge the gap between 8 angles and their PWM values read by the magnetic encoder. I didn't come up with this solution myself, but after implementation it worked well and also made clear why it was a better and simpler tool for the problem. It is now a technique I recognize and will use when encountering similar problems in the future.


### Coding

Because I learned most of the controls concepts before starting the building, implementing the angling, PID loops, and gravity compensation of this part was very straightforward.

However, I hit a roadblock in trying to make the arm build momentum to overcome torque and reach angles from specific, high torque force starting points.
My first thought was to implement a swing up state for this, like the ones typically used in inverted pendulums as shown below:

![ALT TEXT](https://storage.ghost.io/c/5b/7b/5b7b4b03-8aa3-41fc-90fc-c26c215e6ac1/content/images/2018/12/freegifmaker.me_2dR21.gif)


This one uses swinging to build kinetic and potential energy and then reach an upright equilibrium point.

I tried implementing something similar but failed, with the friction of the cheaper TT motor messing up calculations and making things too complicated beyond the scope of this project.

Given time constraints and trying to apply what I already knew, I then decided to simply use another feedback loop with gravity compensation to reach a higher angle from the opposite direction, and then switch back to the regular targeting state from there.
And that worked! Momentum was built in the opposite direction overcoming friction and torque forces, allowing the arm to target from more favorable positions. 

Sometimes applying what you do know in new ways can beat trying to replicate other designs blindly, even if they initially seem ideal.

## 🔨 Limitations
There are 2 main limitations in this current build that I consistently got from testing:

### Slight over/undershooting and the ±7° angle range
If the target is 225°, the arm could stop in between 218° to 232°. Overshooting can occur from the Kp (PID's proportional term) and undershooting with Kd (PID's derivative term), but Ki (PID's integral term) then comes to correct it after a while. But after a certain point, the friction of the cheap TT motor makes each Ki speed change harder to maintain consistently, causing the corrections and final angles to not always be perfect. This explains the range.

### Timing could take a bit
Lets use 225° ± 7° again. If the target is greater than 232° and it stalls trying to move upward back to 225° at max speed, it switches to the swing up momentum state to then try again from the other side. It goes from 180° on the other side back but it could still overshoot again due to one of the PID constants. If it does, it goes back to that swing up state again, trying another time until angle readings, friction, or the right momentum contribute to it stopping at the ±7 range.
After that, the Ki term then can take a few seconds to accumulate the error and then correct it to be even closer.

Basically what I'm trying to say is once it gets into the range, a bit of time could have passed since the target was set, thus being another limitation.

## 🔮 What's Next

From here I'd have to tighten the limitations by making a smaller, more precise angle range and further adjusting constants to make sure over/undershooting doesn't happen as much.

Apart from changing the constants, I could add more capture bands in the targeting state code when approaching targets with different speeds so that I could account for how the friction of the TT motor works.

