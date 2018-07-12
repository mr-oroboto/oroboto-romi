#!/usr/bin/env python

import config
import i2c
import udp
import globals

PI_OPTION_FOLLOW_ME_MODE = 0x01

def botLabCallback(data):
    dataString = data.decode('ascii')
    payload = dataString.split()

    if len(payload) < 8:   # name + all option bytes plus one waypoint pair
        print('Received invalid command payload: [%s]' % dataString)
        return

    if payload[0] == config.name:
        print('Ignoring broadcast command from myself')
        return

    maxVelocity = int(payload[1])
    pivotTurnSpeed = int(payload[2])
    optionByte1 = int(payload[3])
    optionByte2 = int(payload[4])
    piOptionByte1 = int(payload[5])

    if piOptionByte1 & PI_OPTION_FOLLOW_ME_MODE:
        globals.followMe = True
    else:
        globals.followMe = False

    globals.followMeWaypoints = []
    globals.followMeMaxVelocity = maxVelocity
    globals.followMePivotTurnSpeed = pivotTurnSpeed

    points = payload[6:]
    waypoints = []

    if len(points) % 2 != 0:
        print('Received invalid waypoint instructions from botLab: [%s]' % dataString)
        return

    waypointCount = int(len(points) / 2)

    print('Received %d waypoints [%s]' % (waypointCount, dataString))

    for i in range(waypointCount):
        waypoint = (int(points[i*2]), int(points[(i*2)+1]))
        waypoints.append(waypoint)

    msg = ('Transit between %d waypoints [maxVelocity: %d, pivotSpeed: %d, opt1: %2X, opt2: %2X, piOpt1: %2X]' % (waypointCount, maxVelocity, pivotTurnSpeed, optionByte1, optionByte2, piOptionByte1))
    print(msg)
    udp.logToBotlab(msg, False)

    transmitSegments = i2c.buildTransmitSegments(waypoints, maxVelocity, pivotTurnSpeed, optionByte1, optionByte2)
    i2c.registerTransmitSegments(transmitSegments)

    return 0

if __name__ == '__main__':
    i2c.registerI2CSlave(config.i2cSlaveAddr)

    udp.sendPong('255.255.255.255', config.udpBotLabPort, config.name, config.colour)
    udp.listenForBotLab(config.udpLocalPort, config.udpBotLabPort, config.name, config.colour, botLabCallback)

    i2c.stopI2CSlave()