#!/usr/bin/env python

import time
import pigpio
import sys
import struct
import math
import socket

I2C_ADDR=0x13
UDP_ADDR="192.168.0.12"
UDP_PORT=50607

transmitSegments = []
segmentsSent = 0

def logToBotlab(msg):
    """Sends a log back to botLab.

    :param msg: The log message.
    """
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.sendto(bytes(msg, 'ascii'), (UDP_ADDR, UDP_PORT))

def buildTransmitSegments(waypoints):
    """Builds an array of I2C segments to transmit a list of waypoints to the bot.

    :param waypoints: An array of (x,y) tuples specifying the waypoints to visit.
    :return: An array of I2C segments to transmit.
    """
    transmitSegments = []

    if len(waypoints) > 16:
        print("only 16 waypoints can be defined!")
        return transmitSegments

    print("processing %d waypoints" % len(waypoints))

    currentSegment = bytearray(struct.pack(">BBBBB", 0xA0, 0xA2, 0XA2, len(waypoints), len(waypoints)))

    for i in range(len(waypoints)):
        waypoint = waypoints[i]
        print("processing waypoint %d: (%d,%d)" % (i, waypoint[0], waypoint[1]))

        currentSegment.extend(struct.pack(">hh", waypoint[0], waypoint[1]))

        if len(currentSegment) == 9 or (i == len(waypoints) - 1):
            print("closing current segment")
            for j in range(9 - len(currentSegment)):
                print(" + packing")
                currentSegment.extend(struct.pack(">B", 0x00))
            currentSegment.extend(struct.pack(">B", 0xA1))
            transmitSegments.append(currentSegment)
            currentSegment = bytearray(struct.pack(">B", 0xA0))

    print("created %d transmit segments" % len(transmitSegments))

    return transmitSegments

def i2c(id, tick):
    """Handle an I2C interrupt to either send or receive data from the bot.
    """

    global pi
    global transmitSegments
    global segmentsSent

    s, b, d = pi.bsc_i2c(I2C_ADDR)
    if b:
        if d[0] == ord('p') and segmentsSent < len(transmitSegments):
            """CMD: PING: Send the next waypoint I2C segment."""
            s, b, d = pi.bsc_i2c(I2C_ADDR, transmitSegments[segmentsSent])
            segmentsSent += 1

        elif d[0] == ord('r'):
            """CMD: REPORT: Receive the next pose snapshot report."""
            if b == 11:
                x = struct.unpack_from(">h", d, 2)[0]
                y = struct.unpack_from(">h", d, 4)[0]
                heading = struct.unpack_from("b", d, 6)[0]
                headingFloat = struct.unpack_from("B", d, 7)[0]
                distanceToObstacle = struct.unpack_from(">H", d, 8)[0]

                if heading >= 0:
                    heading += (headingFloat / 100.0)
                else:
                    heading -= (headingFloat / 100.0)

                obstacleX = x + (distanceToObstacle * math.cos(heading))
                obstacleY = y + (distanceToObstacle * math.sin(heading))

                print("%d\t%d\t%f\t%d\t%d\t%d" % (x, y, heading, distanceToObstacle, obstacleX, obstacleY))
                msg = "%.2f\t%.2f\t%.2f\t%.2f\t%.2f" % (0, x, y, obstacleX, obstacleY)
                logToBotlab(msg)
#           else:
#             print("read %d bytes for snapshot report, too short!" % b)
#       else:
#             print("read unknown command %x" % d[0])
#   else:
#       print("received empty i2c interrupt")


if __name__ == '__main__':
    pi = pigpio.pi()

    if not pi.connected:
        print('Cannot initialize pigpio!')
        exit()

    """Build the array of I2C segments to send for these waypoints"""
#    transmitSegments = buildTransmitSegments([(100,100),(-200,500),(233,992)])
#    transmitSegments = buildTransmitSegments([(50,0),(50,50),(0,50),(0,0)])
#    transmitSegments = buildTransmitSegments([(100,0),(100,100),(0,100),(0,0)])
    transmitSegments = buildTransmitSegments([(50,50),(100,0)])
#    transmitSegments = buildTransmitSegments([(100,0),(100,100)])
#    transmitSegments = buildTransmitSegments([(100,0),(100,-100),(0,-100),(0,0)])
#    transmitSegments = buildTransmitSegments([(100,0)])
#    transmitSegments = buildTransmitSegments([(50,0),(50,50),(0,50)])

    e = pi.event_callback(pigpio.EVENT_BSC, i2c)    # register for I2C callbacks
    pi.bsc_i2c(I2C_ADDR)                            # configure BSC device as I2C slave to Arduino

    time.sleep(60)                                  # transmit and receive over a 60 second window
    e.cancel()

    pi.bsc_i2c(0)                                   # Disable BSC peripheral
    pi.stop()