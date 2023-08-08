#!/usr/bin/env python

import RPi.GPIO as GPIO
import time
import rospy

output_pin = 33  # Set the output pin to 33 for JETSON_NX

def main():
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(output_pin, GPIO.OUT, initial=GPIO.HIGH)
    p = GPIO.PWM(output_pin, 50)
    val = 100 # start as 100
    p.start(val)

    rospy.init_node('servo_controller')
    
    try:
        while not rospy.is_shutdown():
            servo_position = rospy.get_param('servo_position', 100)
            if 0 <= servo_position <= 100:
                val = servo_position
                p.ChangeDutyCycle(val)
                print("PWM duty cycle set to", val)
            else:
                print("Error: Input must be between 0 and 100.")
            
            time.sleep(1)
    finally:
        p.stop()
        GPIO.cleanup()
        print("PWM stopped and GPIO cleaned up.")

if __name__ == '__main__':
    main()
