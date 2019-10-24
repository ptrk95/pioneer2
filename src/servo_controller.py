#!/usr/bin/env python
import rospy
from pioneer2.msg import control

import Adafruit_PCA9685
import logging

class servo_controller:
    def __init__(self):
	
	logging.basicConfig(level=logging.DEBUG)
        self.pwm = Adafruit_PCA9685.PCA9685(address=0x40)
        self.freq = 50;
        self.pwm.set_pwm_freq(self.freq)
        self.image_width = rospy.get_param("/visualizer/image_width", default=480)

        rospy.Subscriber('visualizer/servo_control', control, self.callback)

    def angle_to_ms(self, angle):
        if(angle < 0):
            angle = angle + 360;
	ms = (0.5+angle*(2./360)) # dont forget the dot "." before "/" !
	print(angle)
	print(ms)
        return ms;

# 1.5ms is middle, 1.0 camera looks right, 2 camera looks left
                
# between 0.5ms and 2.5ms
    def ms_to_pulse(self, ms):
        ms = ms * 0.001 # from sec to ms
        pulse = self.freq * ms * 4096;
        return pulse;
        
    # channel 0 tilt and channel 1 pan
    def set_angle(self, angle, channel):
        ms = self.angle_to_ms(angle)
        pulse = self.ms_to_pulse(ms)
        pulse = int(pulse)
	print(angle)
	print(pulse)
	print(ms)
        self.pwm.set_pwm(channel, 0, pulse)
        
    def callback(self, data):
        if(data.msg == "tilt_camera"):
            if(data.num >= 90):
                self.set_angle( data.num, 0)
            else:
                print("Set tilt angle to 90 or above!")
        if(data.msg == "pan_camera"):
            self.set_angle(data.num, 1)

    def __del__(self):
	self.pwm = Adafruit_PCA9685.PCA9685(address=0x40)
 
def main():
    serv_cont = servo_controller()
    rospy.init_node('servo_controller', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
         print("Shutting down")

if __name__ == '__main__':
    main()
