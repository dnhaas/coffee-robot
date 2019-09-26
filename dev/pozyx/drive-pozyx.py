#!/usr/bin/env python3

from __future__ import print_function
from __future__ import division

from time import sleep

from pypozyx import (POZYX_POS_ALG_UWB_ONLY, POZYX_3D, Coordinates, EulerAngles, Acceleration, POZYX_SUCCESS, PozyxConstants, version,
                     DeviceCoordinates, PozyxSerial, get_first_pozyx_serial_port, SingleRegister, DeviceList, PozyxRegisters)
from pythonosc.udp_client import SimpleUDPClient
from pypozyx.tools.version_check import perform_latest_version_check

import odrive
from odrive.enums import *

from fibre import protocol

import time
import math

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

class RobotStopException(Exception):
    """
    The simplest possible subclass of Exception, we'll raise this if we want to stop the robot
    for any reason. Creating a custom exception like this makes the code more readable later.
    """
    pass

class ReadyToLocalize(object):
    """Continuously calls the Pozyx positioning function and prints its position."""

    def __init__(self, pozyx, osc_udp_client, anchors, algorithm=POZYX_POS_ALG_UWB_ONLY, dimension=POZYX_3D, height=1000, remote_id=None):
        self.pozyx = pozyx
        self.osc_udp_client = osc_udp_client

        self.anchors = anchors
        self.algorithm = algorithm
        self.dimension = dimension
        self.height = height
        self.remote_id = remote_id
        self.measurement = 0
        self.totalPosition = Coordinates()

    def setup(self):
        """Sets up the Pozyx for positioning by calibrating its anchor list."""
        print("------------POZYX POSITIONING V{} -------------".format(version))
        print("")
        print("- System will manually configure tag")
        print("")
        print("- System will auto start positioning")
        print("")
        if self.remote_id is None:
            self.pozyx.printDeviceInfo(self.remote_id)
        else:
            for device_id in [None, self.remote_id]:
                self.pozyx.printDeviceInfo(device_id)
        print("")
        print("------------POZYX POSITIONING V{} -------------".format(version))
        print("")

        self.setAnchorsManual(save_to_flash=False)
        self.printPublishConfigurationResult()

    def printPublishPosition(self, position):
        """Prints the Pozyx's position and possibly sends it as a OSC packet"""
        network_id = self.remote_id
        if network_id is None:
            network_id = 0
        print("POS ID {}, x(mm): {pos.x} y(mm): {pos.y} z(mm): {pos.z}".format(
            "0x%0.4x" % network_id, pos=position))
        if self.osc_udp_client is not None:
            self.osc_udp_client.send_message(
                "/position", [network_id, int(position.x), int(position.y), int(position.z)])

    def printPublishErrorCode(self, operation):
        """Prints the Pozyx's error and possibly sends it as a OSC packet"""
        error_code = SingleRegister()
        network_id = self.remote_id
        if network_id is None:
            self.pozyx.getErrorCode(error_code)
            print("LOCAL ERROR %s, %s" % (operation, self.pozyx.getErrorMessage(error_code)))
            if self.osc_udp_client is not None:
                self.osc_udp_client.send_message("/error", [operation, 0, error_code[0]])
            return
        status = self.pozyx.getErrorCode(error_code, self.remote_id)
        if status == POZYX_SUCCESS:
            print("ERROR %s on ID %s, %s" %
                  (operation, "0x%0.4x" % network_id, self.pozyx.getErrorMessage(error_code)))
            if self.osc_udp_client is not None:
                self.osc_udp_client.send_message(
                    "/error", [operation, network_id, error_code[0]])
        else:
            self.pozyx.getErrorCode(error_code)
            print("ERROR %s, couldn't retrieve remote error code, LOCAL ERROR %s" %
                  (operation, self.pozyx.getErrorMessage(error_code)))
            if self.osc_udp_client is not None:
                self.osc_udp_client.send_message("/error", [operation, 0, -1])
            # should only happen when not being able to communicate with a remote Pozyx.

    def setAnchorsManual(self, save_to_flash=False):
        """Adds the manually measured anchors to the Pozyx's device list one for one."""
        status = self.pozyx.clearDevices(remote_id=self.remote_id)
        for anchor in self.anchors:
            status &= self.pozyx.addDevice(anchor, remote_id=self.remote_id)
        if len(self.anchors) > 4:
            status &= self.pozyx.setSelectionOfAnchors(PozyxConstants.ANCHOR_SELECT_AUTO, len(self.anchors),
                                                       remote_id=self.remote_id)

        if save_to_flash:
            self.pozyx.saveAnchorIds(remote_id=self.remote_id)
            self.pozyx.saveRegisters([PozyxRegisters.POSITIONING_NUMBER_OF_ANCHORS], remote_id=self.remote_id)
        return status

    def printPublishConfigurationResult(self):
        """Prints and potentially publishes the anchor configuration result in a human-readable way."""
        list_size = SingleRegister()

        self.pozyx.getDeviceListSize(list_size, self.remote_id)
        print("List size: {0}".format(list_size[0]))
        if list_size[0] != len(self.anchors):
            self.printPublishErrorCode("configuration")
            return
        device_list = DeviceList(list_size=list_size[0])
        self.pozyx.getDeviceIds(device_list, self.remote_id)
        print("Calibration result:")
        print("Anchors found: {0}".format(list_size[0]))
        print("Anchor IDs: ", device_list)

        for i in range(list_size[0]):
            anchor_coordinates = Coordinates()
            self.pozyx.getDeviceCoordinates(device_list[i], anchor_coordinates, self.remote_id)
            print("ANCHOR, 0x%0.4x, %s" % (device_list[i], str(anchor_coordinates)))
            if self.osc_udp_client is not None:
                self.osc_udp_client.send_message(
                    "/anchor", [device_list[i], int(anchor_coordinates.x), int(anchor_coordinates.y), int(anchor_coordinates.z)])
                sleep(0.025)

    def printPublishAnchorConfiguration(self):
        """Prints and potentially publishes the anchor configuration"""
        for anchor in self.anchors:
            print("ANCHOR,0x%0.4x,%s" % (anchor.network_id, str(anchor.coordinates)))
            if self.osc_udp_client is not None:
                self.osc_udp_client.send_message(
                    "/anchor", [anchor.network_id, int(anchor.coordinates.x), int(anchor.coordinates.y), int(anchor.coordinates.z)])
                sleep(0.025)

    def printOrientationAcceleration(self):
        orientation = EulerAngles()
        acceleration = Acceleration()
        self.pozyx.getEulerAngles_deg(orientation, self.remote_id)
        self.pozyx.getAcceleration_mg(acceleration, self.remote_id)
        print("Orientation: %s, acceleration: %s" % (str(orientation), str(acceleration)))

    def getHeading(self):
        orientation = EulerAngles()
        self.pozyx.getEulerAngles_deg(orientation, self.remote_id)
        return orientation.heading

    def getCurrentPosition(self):
        position = Coordinates()
        totalPosition = Coordinates()

        measurement = 0
        numberOfMeasurements = 15
        pozyxFailCount = 0

        while (measurement < numberOfMeasurements):
            status = self.pozyx.doPositioning(
                position, self.dimension, self.height, self.algorithm, remote_id=self.remote_id)
            if status == POZYX_SUCCESS:
                totalPosition.x = totalPosition.x + position.x
                totalPosition.y = totalPosition.y + position.y
                totalPosition.z = totalPosition.z + position.z
                measurement = measurement + 1                    
            else:
                pozyxFailCount = pozyxFailCount + 1

        print("Failed Pozyx measurements: ",pozyxFailCount)
        totalPosition.x = totalPosition.x / numberOfMeasurements
        totalPosition.y = totalPosition.y / numberOfMeasurements
        totalPosition.z = totalPosition.z / numberOfMeasurements

        return totalPosition

    def getTargetData(self, currentPosition, targetPosition):
        positionError = Coordinates()

        positionError.x = targetPosition.x - currentPosition.x
        positionError.y = targetPosition.y - currentPosition.y

        targetDistance = math.sqrt(math.pow(positionError.x, 2) + math.pow(positionError.y, 2))
        
        targetAngle = 0
        if positionError.x > 0:
            if positionError.y > 0:
                targetAngle = 90 - math.degrees(math.atan(positionError.x / positionError.y))
            else:
                targetAngle = 270 - math.degrees(math.atan(positionError.x / positionError.y))
        else:
            if positionError.y > 0:
                targetAngle = 90 - math.degrees(math.atan(positionError.x / positionError.y))
            else:
                targetAngle = 270 - math.degrees(math.atan(positionError.x / positionError.y))
        
        return [targetDistance, targetAngle]

try:
    if __name__ == "__main__":
        # Check for the latest PyPozyx version. Skip if this takes too long or is not needed by setting to False.
        check_pypozyx_version = True
        if check_pypozyx_version:
            perform_latest_version_check()

        # shortcut to not have to find out the port yourself
        serial_port = get_first_pozyx_serial_port()
        if serial_port is None:
            print("No Pozyx connected. Check your USB cable or your driver!")
            quit()

        remote_id = 0x6e30                 # remote device network ID
        remote = False                   # whether to use a remote device
        if not remote:
            remote_id = None

        # enable to send position data through OSC
        use_processing = False

        # configure if you want to route OSC to outside your localhost. Networking knowledge is required.
        ip = "127.0.0.1"
        network_port = 8888

        osc_udp_client = None
        if use_processing:
            osc_udp_client = SimpleUDPClient(ip, network_port)

        # necessary data for calibration, change the IDs and coordinates yourself according to your measurement
        anchors = [DeviceCoordinates(0x6E63, 1, Coordinates(0, 0, 2220)),
                DeviceCoordinates(0x694A, 1, Coordinates(2200, 5000, 1560)),
                DeviceCoordinates(0x6E57, 1, Coordinates(6500, 5000, 2430)),
                DeviceCoordinates(0x6957, 1, Coordinates(5350, 0, 1520))]

        # positioning algorithm to use, other is PozyxConstants.POSITIONING_ALGORITHM_TRACKING
        algorithm = PozyxConstants.POSITIONING_ALGORITHM_UWB_ONLY
        # positioning dimension. Others are PozyxConstants.DIMENSION_2D, PozyxConstants.DIMENSION_2_5D
        dimension = PozyxConstants.DIMENSION_3D
        # height of device, required in 2.5D positioning
        height = 1000

        pozyx = PozyxSerial(serial_port)
        r = ReadyToLocalize(pozyx, osc_udp_client, anchors, algorithm, dimension, height, remote_id)
        r.setup()

        # TARGET POSITION
        targetPosition = Coordinates(3000,3000,0)

        # DESIRED ACCURACY
        distanceAccuracy = 500
        angleAccuracy = 10

        while True:
            try:
                # Bind to any available joystick, this will use whatever's connected as long as the library
                # supports it.
                with ControllerResource(dead_zone=0.1, hot_zone=0.2) as joystick:
                    print('Boojaah!, controller found, press Analog button to stop robot.')
                    print(joystick.controls)
                    # Loop until the joystick disconnects, or we deliberately stop by raising a
                    # RobotStopException
                    while joystick.connected:
                        joystick.check_presses()
                        # Print out any buttons that were pressed, if we had any
                        if joystick.has_presses:
                            print(joystick.presses)
                        # If home was pressed, raise a RobotStopException to bail out of the loop
                        # Home is the Analog button
                        if 'home' in joystick.presses:
                            raise RobotStopException()
                        if 'start' in joystick.presses:
                            try:
                                my_drive.reboot()
                            except protocol.ChannelBrokenException:
                                init()

                        currentPosition = r.getCurrentPosition()
                        currentAngle = r.getHeading()

                        [targetDistance, targetAngle] = r.getTargetData(currentPosition, targetPosition)

                        if targetDistance > distanceAccuracy:
                            angleError = targetAngle - currentAngle
                            if angleError > 180:
                                angleError = 360 - angleError
                            else:
                                if angleError < -180:
                                    angleError = 360 + angleError

                            if abs(angleError) > angleAccuracy:
                                if angleError > 0:
                                    # turn right
                                    set_speeds(0.75 * targetDistance,-0.75 * targetDistance)
                                    print('Right',angleError)
                                else:
                                    # turn left
                                    set_speeds(-0.75 * targetDistance,0.75 * targetDistance)
                                    print('Left',angleError)

                            else:
                                # drive forward
                                set_speeds(1.5 * targetDistance,1.5 * targetDistance)
                                print('Drive forward: ',targetDistance)
                        else:
                            # position reached
                            stop_motors()
                            print('Position reached [dis: ',targetDistance,'; ang: ',targetAngle,']')
                    
                    stop_motors()
            except IOError:
                # We get an IOError when using the ControllerResource if we don't have a controller yet,
                # so in this case we just wait a second and try again after printing a message.
                print('No controller found yet')
                sleep(1)

except RobotStopException:
    # This exception will be raised when the home button is pressed, at which point we should
    # stop the motors.
    stop_motors()   