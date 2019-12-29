#!/usr/bin/env python
import rospy
from pioneer2.msg import control

from std_srvs.srv import Trigger, TriggerResponse

import Adafruit_PCA9685
from threading import Timer

wait = False
wait_t = False

class Timer_(object):

    def __init__(self, time, sleeper, *args, **kwargs):
        self.args = args
        self.kwargs = kwargs
        self.timer = None
        self.time = time
        self.sleeper = sleeper

    def callback(self):
        global wait, wait_t
        if(self.sleeper):
            wait = False
        else:
            wait_t = False

    def cancel(self):
        self.timer.cancel()

    def start(self):
        self.timer = Timer(self.time, self.callback)
        self.timer.start()

class servo_controller:
    def __init__(self):
        self.pwm = Adafruit_PCA9685.PCA9685(address=0x40)
        self.freq = 50
        self.pwm.set_pwm_freq(self.freq)
        self.set_standard_pos()
        #self.pub_pan_angle = rospy.Publisher('servo_controller/pan_angle', control, queue_size=1)
        rospy.Subscriber('master/servo_control_pan', control, self.callback)
        rospy.Subscriber('master/servo_control_tilt', control, self.callback)
        self.timer = Timer_(1.0, True)
        self.timer_t = Timer_(1.0, False)
        self.get_pan_pos = rospy.Service("servo_controller/pan_angle", Trigger, self.trigger_response)

    def trigger_response(self, request):
        return TriggerResponse(success=True, message= str(self.pos_pan))
    
    def set_standard_pos(self):
        self.pos_tilt = rospy.get_param('servo_controller/std_tilt', default=0)
        self.pos_pan  = rospy.get_param('servo_controller/std_pan', default=0)
        self.pan_camera(self.pos_pan)
        self.tilt_camera(self.pos_tilt)

# channel 0 tilt and channel 1 pan
    def angle_to_ms(self, angle, channel):
        if(channel == 0):
            ms = (0.67+angle*(0.89/90)) # dont forget the dot "." before "/" !
        elif(channel == 1):
            if(angle <= 0):
                angle = -angle
                ms = 1.56 - angle*(0.9/90)
            elif(angle > 0):
                ms = 1.56 + angle*(1.1/90)
            #angle = angle + 90;
            #ms = (0.67+angle*(0.89/90)) # dont forget the dot "." before "/" 
            #ms = rospy.get_param('test_pulse', default=0.67) # 1.56
        
        return ms

#  1.0 camera looks right, 1.5ms is middle, 2ms camera looks left
                
# between 1ms and 2ms (from datasheet of servos)
    def ms_to_pulse(self, ms):
        ms = ms * 0.001 # from ms to sec
        pulse = self.freq * ms * 4096
        pulse = int(pulse)
        return pulse
        
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
        if(angle >= 0 and angle <= 90):
            self.set_angle(angle, 0)
        else:
            print("Only angles between 0 and 90 allowed to tilt camera.")

    # Tilt camera relative to current position by angle degrees.
    def tilt_camera_rel(self, angle):
        if(self.pos_tilt + angle >= 0 and self.pos_tilt + angle <= 90):
            self.tilt_camera(angle + self.pos_tilt)
            self.pos_tilt = self.pos_tilt + angle
        else:
            print("Reached max or min angle to tilt camera.")

    # Pan camera relative to current position by angle degrees.
    def pan_camera_rel(self, angle):
        if(self.pos_pan + angle >= -80 and self.pos_pan + angle <= 80):
            self.pan_camera(angle + self.pos_pan)
            self.pos_pan = self.pos_pan + angle
        else:
            print("Reached max or min angle to pan camera.")

    def callback(self, data):
        global wait, wait_t
        if(data.msg == "tilt_camera" and not wait_t):
            self.tilt_camera_rel(data.num)
            wait_t = True
            self.timer_t.start()
        if(data.msg == "pan_camera" and not wait):
            self.pan_camera_rel(data.num)
            wait = True
            self.timer.start()
        if(data.msg == "force_0_pan_camera"):
            self.pan_camera(0)
            self.pos_pan = 0
            wait = True
            self.timer.start()
        

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
