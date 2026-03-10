#!/usr/bin/env python3

class SServo:
    TICK_UP = 100
    TICK_DOWN = -100
    POS_UP = -700
    POS_DOWN = 300
    SPEED = 200
    CURRENT_POS = 0
    
    def setup(self):
        from uservice import service
        self.CURRENT_POS = 0
        service.send("robobot/cmd/T0","servo 1 {self.CURRENT_POS} {self.SPEED}")
        print("% SServo:: setup complete")

    def servo_down_one_tick(self):
        from uservice import service
        self.CURRENT_POS += self.TICK_DOWN
        service.send("robobot/cmd/T0","servo 1 {self.TICK_DOWN} {self.SPEED}")
        print(f"% SServo:: servo down one tick: {self.CURRENT_POS}")

    def servo_down_position(self):
        from uservice import service
        self.CURRENT_POS = self.POS_DOWN
        service.send("robobot/cmd/T0","servo 1 300 100")
        print(f"% SServo:: servo down: {self.CURRENT_POS}")

    def servo_up_one_tick(self):
        from uservice import service
        self.CURRENT_POS += self.TICK_UP
        service.send("robobot/cmd/T0","servo 1 {self.TICK_UP} {self.SPEED}")
        print(f"% SServo:: servo up one tick: {self.CURRENT_POS}")

    def servo_up_position(self):
        from uservice import service
        self.CURRENT_POS = self.POS_UP
        service.send("robobot/cmd/T0","servo 1 {self.POS_UP} {self.SPEED}")
        print(f"% SServo:: servo up: {self.CURRENT_POS}")

    def servo_center(self):
        from uservice import service
        self.CURRENT_POS = 0
        service.send("robobot/cmd/T0","servo 1 {self.CURRENT_POS} {self.SPEED}")
        print(f"% SServo:: servo center: {self.CURRENT_POS}")
        
servo = SServo()