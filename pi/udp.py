import sys
import socket

BUFSIZ = 1024

botLabAddr = 0
botLabPort = 0


def listenForBotLab(port, callback):
    global botLabAddr
    global botLabPort

    if botLabPort == 0:
        botLabPort = port

    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(('', port))
    while 1:
        data, addr = s.recvfrom(BUFSIZ)

        if botLabAddr == 0:
            botLabAddr = addr

        callback(data)


def logToBotlab(msg):
    """Sends a log back to botLab.

    :param msg: The log message.
    """

    global botLabAddr
    global botLabPort

    if botLabAddr:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.sendto(bytes(msg, 'ascii'), (botLabAddr, botLabPort))


def sendToAddr(addr, port, msg):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.sendto(msg, (addr, port))