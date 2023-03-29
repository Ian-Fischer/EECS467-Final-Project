#!/usr/bin/env python3
from gpiozero import Device, AngularServo
from gpiozero.pins.pigpio import PiGPIOFactory
import rospy
from slos_bot.srv import ControlClaw, ControlClawResponse

OPEN_ANGLE = -60
CLOSED_CAN_ANGLE = -10
CLOSED_BOX_ANGLE = -20

class ClawController:
    def __init__(self):
        Device.pin_factory = PiGPIOFactory()
        self.servo = AngularServo(12)
        rospy.init_node("claw_controller")
        s = rospy.Service("control_claw", ControlClaw, self.set_claw_angle)
        
    def set_claw_angle(self, req: ControlClaw):
        angle = None
        if req.state == req.OPEN:
            angle = OPEN_ANGLE
        elif req.state == req.CLOSED_CAN:
            angle = CLOSED_CAN_ANGLE
        elif req.state == req.CLOSED_BOX:
            angle = CLOSED_BOX_ANGLE
        else:
            return ControlClawResponse(False)
        
        self.servo.angle = angle
        return ControlClawResponse(self.servo.angle == angle)

if __name__ == "__main__":
    ClawController()
    rospy.spin()
