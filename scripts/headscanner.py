#!/usr/bin/env python2

import rospy
import threading
import RPi.GPIO as GPIO
import time

from pi_distance_scanner.msg import HeadDistance
from pi_distance_scanner.mystepper import Stepper

stepMotor = None

# Define GPIO to use on Pi
GPIO_TRIGGER    = 16
GPIO_ECHO       = 20

MAX_STEP        = 1000
SCAN_COUNT      = 20

def config_gpio():
    global stepMotor
    # Numbering by GPIO number, not header pin numberring
    GPIO.setmode(GPIO.BCM)

    # Use GPIO04 J8-07 |
    #     GPIO17 J8-11 |
    #     GPIO27 J8-13 |
    #     GPIO22 J8-15 | to control stepper motor

    # Config pins
    step_pins = [4, 17, 27, 22]

    for pin in step_pins:
        GPIO.setup(pin, GPIO.OUT, initial=0)
        GPIO.output(pin, 0)

    def write(value):
        if value & 0x01:
            GPIO.output(step_pins[0], 1)
        else:
            GPIO.output(step_pins[0], 0)
        if value & 0x02:
            GPIO.output(step_pins[1], 1)
        else:
            GPIO.output(step_pins[1], 0)
        if value & 0x04:
            GPIO.output(step_pins[2], 1)
        else:
            GPIO.output(step_pins[2], 0)
        if value & 0x08:
            GPIO.output(step_pins[3], 1)
        else:
            GPIO.output(step_pins[3], 0)
    stepMotor = Stepper(write=write, full_step=False, delay=0.001)

    # Set pins as output and input
    GPIO.setup(GPIO_TRIGGER,GPIO.OUT)  # Trigger
    GPIO.setup(GPIO_ECHO,GPIO.IN)      # Echo

    # Set trigger to False (Low)
    GPIO.output(GPIO_TRIGGER, False)


def measure():
    # This function measures a distance
    GPIO.output(GPIO_TRIGGER, True)
    time.sleep(0.00001)
    GPIO.output(GPIO_TRIGGER, False)
    start = time.time()

    while GPIO.input(GPIO_ECHO)==0:
        start = time.time()

    while GPIO.input(GPIO_ECHO)==1:
        stop = time.time()

    elapsed = stop-start
    distance = (elapsed * 34300)/2.0
    return distance

def terminate():
    rospy.loginfo(rospy.get_caller_id() + "headscanner: TERMINATE")
    GPIO.cleanup()

def listener():
    global stepMotor
    rospy.init_node('pi_head_scanner', anonymous=True)

    config_gpio()

    pub = rospy.Publisher('pi_head_distance', HeadDistance, queue_size=10)

    curstep = 0
    curdir = 1
    onestep = MAX_STEP/SCAN_COUNT

    curMess = HeadDistance()

    while not rospy.is_shutdown():
        # scan distance and publish message
        curdist = measure()
        curMess.step = curstep
        curMess.distance = curdist
        pub.publish(curMess)
        rospy.loginfo(rospy.get_caller_id() + " headscanner: send msg step=%d distance=%f" % (curstep, curdist))

        # calculate next step
        curstep += curdir
        if abs(curstep) >= SCAN_COUNT:
            # back
            curdir = -curdir
            curstep += 2 * curdir

        # move the head
        stepMotor.step_relative(onestep * curdir)
        time.sleep(0.02)

    terminate()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass

