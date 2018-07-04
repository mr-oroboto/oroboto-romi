#!/usr/bin/env python

import config
import i2c
import udp

def botLabCallback(data):
    dataString = data.decode('ascii')
    payload = dataString.split()

    if len(payload) < 6:
        print('Received invalid payload from botLab: [%s]' % dataString)
        return

    maxVelocity = int(payload[0])
    pivotTurnSpeed = int(payload[1])
    optionByte1 = int(payload[2])
    optionByte2 = int(payload[3])

    points = payload[4:]
    waypoints = []

    if len(points) % 2 != 0:
        print('Received invalid waypoint instructions from botLab: [%s]' % dataString)
        return

    waypointCount = int(len(points) / 2)

    print('Received %d waypoints [%s]' % (waypointCount, dataString))

    for i in range(waypointCount):
        waypoint = (int(points[i*2]), int(points[(i*2)+1]))
        waypoints.append(waypoint)

    print('Executing transit between waypoints at maxVelocity [%d], pivotTurnSpeed [%d], optionByte1 [%2X], optionByte2 [%2X]' % (maxVelocity, pivotTurnSpeed, optionByte1, optionByte2))

    transmitSegments = i2c.buildTransmitSegments(waypoints, maxVelocity, pivotTurnSpeed, optionByte1, optionByte2)
    i2c.registerTransmitSegments(transmitSegments)

    return 0

if __name__ == '__main__':
    i2c.registerI2CSlave(config.i2cSlaveAddr)
    udp.listenForBotLab(config.udpLocalPort, config.udpBotLabPort, config.name, config.colour, botLabCallback)
    i2c.stopI2CSlave()