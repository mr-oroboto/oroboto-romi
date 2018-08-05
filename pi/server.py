#!/usr/bin/env python

import config
import i2c
import udp
import globals
import enums
import math

def botLabCallback(data):
    dataString = data.decode('ascii')
    payload = dataString.split()

    if len(payload) < 8:   # name + all option bytes plus one waypoint pair
        print('Received invalid command payload: [%s]' % dataString)
        return

    if payload[0] == config.name:
        print('Ignoring broadcast command from myself')
        return

    command = int(payload[1])
    success = False
    errMsg  = ''

    if command == enums.PI_CMD_TRANSIT_WAYPOINTS:
        (success, errMsg) = executeTransit(payload[2:])
    elif command == enums.PI_CMD_SET_BOT_OPTION:
        (success, errMsg) = executeSetBotOption(payload[2:])
    elif command == enums.PI_CMD_ROTATE_AND_DRIVE:
        (success, errMsg) = executeRotateAndDrive(payload[2:], 1)
    elif command == enums.PI_CMD_FIND_OBJECT:
        (success, errMsg) = executeFindObject(payload[2:])
    else:
        errMsg = 'Unknown command'

    if not success:
        print('Failed to execute command [%2X]: %s (payload: %s)' % (command, errMsg, dataString))
        msg = 'Failed to execute cmd [%2X]: %s' % (command, errMsg)
        udp.logToBotlab(msg, False)

    return 0


def executeSetBotOption(commandPayload):
    maxVelocity = int(commandPayload[0])
    pivotTurnSpeed = int(commandPayload[1])
    optionByte1 = int(commandPayload[2])
    optionByte2 = int(commandPayload[3])
    piOptionByte1 = int(commandPayload[4])

    packedOptions = commandPayload[5:]
    waypoints = []

    if len(packedOptions) % 2 != 0:
        return (False, 'Received invalid packed options from botLab')

    packedOptionCount = int(len(packedOptions) / 2)

    print('Received %d packed options [%s]' % (packedOptionCount, packedOptions))

    for i in range(packedOptionCount):
        packedOption = (int(packedOptions[i*2]), int(packedOptions[(i*2)+1]))
        waypoints.append(packedOption)

    msg = ''

    if optionByte1 == enums.BOT_OPTION_CALIBRATE:
        msg = ('Executing bot calibration [pivotSpeed: %d]' % (pivotTurnSpeed))
    elif optionByte1 == enums.BOT_OPTION_RESET_TO_ORIGIN:
        msg = 'Reset to origin'
    elif optionByte1 == enums.BOT_OPTION_SET_PID_PARAMETERS:
        msg = 'Updated PID parameters'
    else:
        return (False, 'Unknown bot option')

    print(msg)
    udp.logToBotlab(msg, False)

    transmitSegments = i2c.buildTransmitSegments(waypoints, maxVelocity, pivotTurnSpeed, optionByte1, optionByte2)
    i2c.registerTransmitSegments(transmitSegments)

    return (True, '')


def executeTransit(commandPayload):
    maxVelocity = int(commandPayload[0])
    pivotTurnSpeed = int(commandPayload[1])
    optionByte1 = int(commandPayload[2])
    optionByte2 = int(commandPayload[3])
    piOptionByte1 = int(commandPayload[4])

    globals.followMeWaypoints = []
    globals.followMeMaxVelocity = maxVelocity
    globals.followMePivotTurnSpeed = pivotTurnSpeed

    points = commandPayload[5:]
    waypoints = []

    if len(points) % 2 != 0:
        return (False, 'Received invalid waypoint instructions from botLab')

    waypointCount = int(len(points) / 2)

    print('Received %d waypoints [%s]' % (waypointCount, points))

    for i in range(waypointCount):
        waypoint = (int(points[i*2]), int(points[(i*2)+1]))
        waypoints.append(waypoint)

    if piOptionByte1 & enums.PI_OPTION_FOLLOW_ME_MODE:
        globals.followMe = True
    else:
        globals.followMe = False

    msg = ('Transit between %d waypoints [maxVelocity: %d, pivotSpeed: %d, opt1: %2X, opt2: %2X, piOpt1: %2X]' % (waypointCount, maxVelocity, pivotTurnSpeed, optionByte1, optionByte2, piOptionByte1))
    print(msg)
    udp.logToBotlab(msg, False)

    globals.currentCommand = enums.PI_CMD_TRANSIT_WAYPOINTS

    transmitSegments = i2c.buildTransmitSegments(waypoints, maxVelocity, pivotTurnSpeed, optionByte1, optionByte2)
    i2c.registerTransmitSegments(transmitSegments)

    return (True, '')


def executeRotateAndDrive(commandPayload, phase):
    maxVelocity = int(commandPayload[0])
    pivotTurnSpeed = int(commandPayload[1])
    optionByte1 = int(commandPayload[2])
    optionByte2 = int(commandPayload[3])
    piOptionByte1 = int(commandPayload[4])
    rotationAngle = int(commandPayload[5])
    distance = int(commandPayload[6])

    waypoints = []

    if phase == 1:
        globals.currentCommand = enums.PI_CMD_ROTATE_AND_DRIVE
        globals.rotateAndDrivePayload = commandPayload

        waypoint = (rotationAngle, 0)
        waypoints.append(waypoint)

        msg = ('R&D P1: Rotating [pivotSpeed: %d, opt1: %2X, opt2: %2X]' % (pivotTurnSpeed, optionByte1, optionByte2))
    elif phase == 2:
        globals.currentCommand = 0
        globals.rotateAndDrivePayload = 0

        if distance:
            targetX = globals.currentX + (distance * math.cos(globals.currentHeading))
            targetY = globals.currentY + (distance * math.sin(globals.currentHeading))

            waypoint = (int(targetX), int(targetY))
            waypoints.append(waypoint)
            optionByte2 = 0   # this carries the rotate command, need to switch back to waypoint transit

            msg = ('R&D P2: Driving %d cm from (%d,%d) to (%d,%d) [maxVelocity: %d]' % (distance, globals.currentX, globals.currentY, targetX, targetY, maxVelocity))
        else:
            return (True, '')   # no drive phase
    else:
        globals.currentCommand = 0
        return (False, 'Unknown phase')

    print(msg)
    udp.logToBotlab(msg, False)

    transmitSegments = i2c.buildTransmitSegments(waypoints, maxVelocity, pivotTurnSpeed, optionByte1, optionByte2)
    i2c.registerTransmitSegments(transmitSegments)

    return (True, '')


def executeFindObject(commandPayload):
    return (True, '')


if __name__ == '__main__':
    i2c.registerI2CSlave(config.i2cSlaveAddr)

    udp.sendPong('255.255.255.255', config.udpBotLabPort, config.name, config.colour)
    udp.listenForBotLab(config.udpLocalPort, config.udpBotLabPort, config.name, config.colour, botLabCallback)

    i2c.stopI2CSlave()