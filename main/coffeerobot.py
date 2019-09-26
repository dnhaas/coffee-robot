#!/usr/bin/env python3
"""
MTA Coffee-robot drive with joystick program 
"""

from __future__ import print_function
from __future__ import division

from time import sleep
import odrive
from odrive.enums import *
import time
import math
from fibre import protocol




def init():
    # Find a connected ODrive (this will block until you connect one)
    print("finding an odrive...")
    global my_drive
    my_drive = odrive.find_any()


    # To read a value, simply read the property
    print("Odrive found, Shit is running at " + str(my_drive.vbus_voltage) + "V")

    my_drive.axis0.controller.config.pos_gain = 1
    print("axis 0 pos_gain is " + str(my_drive.axis0.controller.config.pos_gain))
    my_drive.axis1.controller.config.pos_gain = 1
    print("axis 1 pos_gain is " + str(my_drive.axis1.controller.config.pos_gain))

    my_drive.axis0.controller.config.vel_gain = 0.25 #0.02
    print("axis 0 vel_gain is " + str(my_drive.axis0.controller.config.vel_gain))
    my_drive.axis1.controller.config.vel_gain = 0.25 #0.02
    print("axis 1 vel_gain is " + str(my_drive.axis1.controller.config.vel_gain))

    my_drive.axis0.controller.config.vel_integrator_gain = 1
    print("axis 0 vel_integrator_gain is " + str(my_drive.axis0.controller.config.vel_integrator_gain))
    my_drive.axis1.controller.config.vel_integrator_gain = 1
    print("axis 1 vel_integrator_gain is " + str(my_drive.axis1.controller.config.vel_integrator_gain))


    my_drive.axis0.controller.config.vel_limit = 500
    my_drive.axis1.controller.config.vel_limit = 500

init()

try:
    # Attempt to import the GPIO Zero library. If this fails, because we're running somewhere
    # that doesn't have the library, we create dummy functions for set_speeds and stop_motors which
    # just print out what they'd have done. This is a fairly common way to deal with hardware that
    # may or may not exist!

    # Use GPIO Zero implementation of CamJam EduKit robot (thanks Ben Nuttall/Dave Jones!)
    from gpiozero import CamJamKitRobot

    print('GPIO Zero found')

    # Get the robot instance and the independent motor controllers
    #robot = CamJamKitRobot()
    #motor_left = robot.left_motor
    #motor_right = robot.right_motor

    # Motors are reversed. If you find your robot going backwards, set this to 1
    motor_multiplierL = -1
    motor_multiplierR = 1


    def set_speeds(power_left, power_right):
        """
        As we have an motor hat, we can use the motors

        :param power_left: 
            Power to send to left motor
        :param power_right: 
            Power to send to right motor, will be inverted to reflect chassis layout
        """

        # If one wants to see the 'raw' 0-100 values coming in
        # print("source left: {}".format(power_left))
        # print("source right: {}".format(power_right))

        # Take the 0-100 inputs down to 0-1 and reverse them if necessary
        power_left = (motor_multiplierL * power_left) / 100
        power_right = (motor_multiplierR * power_right) / 100

        # Print the converted values out for debug
        # print("left: {}".format(power_left))
        # print("right: {}".format(power_right))
        
        
        #Define to wich parameter the power has to be assigned
        my_drive.axis0.controller.vel_setpoint = (power_left)
        my_drive.axis1.controller.vel_setpoint = (power_right)

    def stop_motors():
        # Turn both motors off
        my_drive.axis0.controller.vel_setpoint = 0
        my_drive.axis1.controller.vel_setpoint = 0

except ImportError:

    print('GPIO Zero not found, using dummy functions.')

    def set_speeds(power_left, power_right):
        """
        No motor hat - print what we would have sent to it if we'd had one.
        """
        print('DEBUG Left: {}, Right: {}'.format(power_left, power_right))
        sleep(0.3)

    def stop_motors():
        """
        No motor hat, so just print a message.
        """
        print('DEBUG Motors stopping')

# All we need, as we don't care which controller we bind to, is the ControllerResource
from approxeng.input.selectbinder import ControllerResource


# Enable logging of debug messages, by default these aren't shown
# import logzero
# logzero.setup_logger(name='approxeng.input', level=logzero.logging.DEBUG)

class RobotStopException(Exception):
    """
    The simplest possible subclass of Exception, we'll raise this if we want to stop the robot
    for any reason. Creating a custom exception like this makes the code more readable later.
    """
    pass


def mixer(yaw, throttle, max_power=20000):
    """
    Mix a pair of joystick axes, returning a pair of wheel speeds. This is where the mapping from
    joystick positions to wheel powers is defined, so any changes to how the robot drives should
    be made here, everything else is really just plumbing.
    
    :param yaw: 
        Yaw axis value, ranges from -1.0 to 1.0
    :param throttle: 
        Throttle axis value, ranges from -1.0 to 1.0
    :param max_power: 
        Maximum speed that should be returned from the mixer, defaults to 100
    :return: 
        A pair of power_left, power_right integer values to send to the motor driver
    """
    left = throttle + yaw
    right = throttle - yaw
    scale = float(max_power) / max(1, abs(left), abs(right))
    return int(left * scale), int(right * scale)


# Outer try / except catches the RobotStopException we just defined, which we'll raise when we want to
# bail out of the loop cleanly, shutting the motors down. We can raise this in response to a button press
try:
    while True:
        # Inner try / except is used to wait for a controller to become available, at which point we
        # bind to it and enter a loop where we read axis values and send commands to the motors.
        try:
            # Bind to any available joystick, this will use whatever's connected as long as the library
            # supports it.
            with ControllerResource(dead_zone=0.1, hot_zone=0.2) as joystick:
                print('Boojaah!, controller found, press Analog button to exit, use left stick to cross keihard door het kantoor! .')
                print(joystick.controls)
                # Loop until the joystick disconnects, or we deliberately stop by raising a
                # RobotStopException
                while joystick.connected:
                    # Get joystick values from the left analogue stick
                    x_axis, y_axis = joystick['lx', 'ly']
                    # Get power from mixer function
                    power_left, power_right = mixer(yaw=x_axis, throttle=y_axis)
                    # Set motor speeds
                    set_speeds(power_left, power_right)
                    # Get a ButtonPresses object containing everything that was pressed since the last
                    # time around this loop.
                    joystick.check_presses()
                    # Print out any buttons that were pressed, if we had any
                    if joystick.has_presses:
                        print(joystick.presses)
                    # If home was pressed, raise a RobotStopException to bail out of the loop
                    # Home is the Analog button
                    if 'home' in joystick.presses:
                        raise RobotStopException()
                    if 'select'in joystick.presses:
                        # To read a value, simply read the property
                        print("Odrive online, running at " + str(my_drive.vbus_voltage) + "V")
                    if 'start' in joystick.presses:
                        try:
                            my_drive.reboot()
                        except protocol.ChannelBrokenException:
                            init()
                    if 'l1'in joystick.presses:
                        print("######## Axis 0 MOTOR settings ######")
                        print(str(my_drive.axis0.motor))
                    if 'l2'in joystick.presses:
                        print("######## Axis 0 ENCODER settings ######")
                        print(str(my_drive.axis0.encoder))
                    if 'r1'in joystick.presses:
                        print("######## Axis 1 MOTOR settings ######")
                        print(str(my_drive.axis1.motor))
                    if 'r2'in joystick.presses:
                        print("######## Axis 1 ENCODER settings ######")
                        print(str(my_drive.axis1.encoder))
                        
        except IOError:
            # We get an IOError when using the ControllerResource if we don't have a controller yet,
            # so in this case we just wait a second and try again after printing a message.
            print('No controller found yet')
            sleep(1)
except RobotStopException:
    # This exception will be raised when the home button is pressed, at which point we should
    # stop the motors.
    stop_motors()




