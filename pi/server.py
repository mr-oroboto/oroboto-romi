#!/usr/bin/env python

import config
import i2c
import udp

UDP_ADDR="192.168.0.12"

def botLabCallback(data):
    dataString = data.decode('ascii')
    points = dataString.split()
    waypoints = []

    if len(points) % 2 != 0 or len(points) == 0:
        print('Received invalid waypoint instructions from botLab: [%s]' % dataString)
        return

    waypointCount = int(len(points) / 2)

    print('Received %d waypoints [%s]' % (waypointCount, dataString))

    for i in range(waypointCount):
        waypoint = (int(points[i]), int(points[i+1]))
        waypoints.append(waypoint)

    transmitSegments = i2c.buildTransmitSegments(waypoints)
    i2c.registerTransmitSegments(transmitSegments)

    return 0

if __name__ == '__main__':
    i2c.registerI2CSlave(config.i2cSlaveAddr)
    udp.listenForBotLab(config.udpBotLabPort, botLabCallback)
    i2c.stopI2CSlave()