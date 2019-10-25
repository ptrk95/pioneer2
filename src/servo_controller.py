#!/usr/bin/env python
import rospy
from pioneer2.msg import control

import Adafruit_PCA9685

class servo_controller:
    def __init__(self):
        self.pwm = Adafruit_PCA9685.PCA9685(address=0x40)
        self.freq = 50;
        self.pwm.set_pwm_freq(self.freq)

        rospy.Subscriber('visualizer/servo_control', control, self.callback)

    def angle_to_ms(self, angle, channel):
        if(channel == 0):
            ms = (1+angle*(1./180)) # dont forget the dot "." before "/" !
            ms = 2.48
        elif(channel == 1):
            angle = angle + 90;
            ms = (1+angle*(1./180)) # dont forget the dot "." before "/" !
        print(angle)
        print(ms)
        
        return ms;

#  1.0 camera looks right, 1.5ms is middle, 2ms camera looks left
                
# between 1ms and 2ms (from datasheet of servos)
    def ms_to_pulse(self, ms):
        ms = ms * 0.001 # from ms to sec
        pulse = self.freq * ms * 4096;
        pulse = int(pulse)
        return pulse;
        
    # channel 0 tilt and channel 1 pan
    def set_angle(self, angle, channel):
        ms = self.angle_to_ms(angle, channel)
        pulse = self.ms_to_pulse(ms)
        self.pwm.set_pwm(channel, 0, pulse)
    
    def pan_camera(self, angle):
        if(angle >= -90 and angle <= 90):
            self.set_angle(angle, 1)
        else:
            print("Only angles between -90 and 90 allowed to pan camera.")
        
    def tilt_camera(self, angle):
        if(angle >= 0 and angle <= 180):
            self.set_angle(angle, 0)
        else:
            print("Only angles between 0 and 180 allowed to tilt camera.")
        
    def callback(self, data):
        if(data.msg == "tilt_camera"):
            self.tilt_camera(data.num)
        if(data.msg == "pan_camera"):
            self.pan_camera(data.num)

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
