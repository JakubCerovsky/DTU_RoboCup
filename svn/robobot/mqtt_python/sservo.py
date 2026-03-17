#!/usr/bin/env python3

# This file operates the front servo of the robobot. It is used to control the position of the servo, which can be adjusted within a specified range. 
# The setup method initializes the servo to a default position, while the servo_change_position method allows for changing the position of the servo based on input. The servo_center method resets the servo to its center position.
# The values for the position can be anywhere between 250 and -900 (positive values move the servo down, negative values move it up). The speed of the servo can also be adjusted, with a default value of 200.
# Position limits were decided based on the physical constraints of the servo combined with the 3D printed front arm of the robot.

class SServo:
    MAX_POS_UP = -900
    MAX_POS_DOWN = 210
    SPEED = 200
    
    def setup(self):
        from uservice import service
        service.send("robobot/cmd/T0",f"servo 1 {self.MAX_POS_UP} {self.SPEED}")
        print("% SServo:: setup complete")

    def servo_change_position(self, pos, speed=None):
        from uservice import service
        if pos < self.MAX_POS_UP:
            pos = self.MAX_POS_UP
        elif pos > self.MAX_POS_DOWN:
            pos = self.MAX_POS_DOWN
        if speed is not None:
            self.SPEED = speed
        service.send("robobot/cmd/T0",f"servo 1 {pos} {self.SPEED}")
        print(f"% SServo:: servo center: {pos}")

    def servo_center(self):
        from uservice import service
        service.send("robobot/cmd/T0",f"servo 1 100 {self.SPEED}")
        print(f"% SServo:: servo center: 100")
        
servo = SServo()