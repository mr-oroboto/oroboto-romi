import socket

BUFSIZ = 1024

botLabAddr = 0
botLabPort = 0


def listenForBotLab(listenPort, useBotLabPort, botName, botColour, callback):
    global botLabAddr
    global botLabPort

    if botLabPort == 0:
        botLabPort = useBotLabPort

    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(('', listenPort))
    while 1:
        data, addr = s.recvfrom(BUFSIZ)

        if botLabAddr == 0:
            botLabAddr = addr[0]

        if len(data) == 4 and data.decode('ascii') == 'ping':
            sendPong(botLabAddr, botLabPort, botName, botColour)
        else:
            callback(data)

def sendPong(botLabAddr, botLabPort, botName, botColour):
    if botLabAddr == '255.255.255.255':
        sendToAddr(botLabAddr, botLabPort, bytes('pong ' + botName + ' ' + botColour, 'ascii'), True)
    else:
        sendToAddr(botLabAddr, botLabPort, bytes('pong ' + botName + ' ' + botColour, 'ascii'), False)


def logToBotlab(msg, msgIsPoseSnapshot):
    """Sends a log back to botLab.

    :param msg: The log message.
    """

    global botLabAddr
    global botLabPort

    msg = '# ' + msg if msgIsPoseSnapshot else '> ' + msg

    if botLabAddr:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.sendto(bytes(msg, 'ascii'), (botLabAddr, botLabPort))


def sendToAddr(addr, port, msg, broadcast):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    if broadcast:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    sock.sendto(msg, (addr, port))