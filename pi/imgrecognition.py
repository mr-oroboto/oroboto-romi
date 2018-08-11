import time
import boto3
import config
import udp
import socket
import struct

from PIL import Image
from picamera import PiCamera

camera = None
rekognitionClient = None
imageServerSocket = None

def detectObjectInSnapshot(objectName, uploadSnapshots):
    global camera
    global rekognitionClient

    if not rekognitionClient:
        rekognitionClient = boto3.client(
            'rekognition',
            region_name=config.awsRegionName,
            aws_access_key_id=config.awsAccessKeyId,
            aws_secret_access_key=config.awsAccessKeySecret
        )

    if not camera:
        camera = PiCamera()
        camera.resolution = (1024, 768)
        camera.rotation = 180
        camera.start_preview()
        time.sleep(2)           # let AWB and auto-gain settle

    timestamp = time.time()
    image_name = '%s/%u.jpg' % (config.imagePath, timestamp)
    image_name_cropped = '%s/%u-crop.jpg' % (config.imagePath, timestamp)

    camera.capture(image_name, resize=(320,240))

    image = Image.open(image_name)
    cropped = image.crop((53, 0, 267, 240))
    cropped.save(image_name_cropped)

    fd = open(image_name_cropped, 'rb')
    bytes = fd.read()
    response = rekognitionClient.detect_labels(
        Image={
            'Bytes': bytes
        },
        MaxLabels=10,
        MinConfidence=0.5
    )
    fd.close()

    if uploadSnapshots:
        uploadSnapshot(image_name_cropped)

    labels = ''

    if response['ResponseMetadata']['HTTPStatusCode'] == 200:
        for label in response['Labels']:
            labels += (' %s' % label['Name'].lower())

            if objectNameMatchesLabel(objectName, label['Name']):
                return (True, '')

    return (False, labels)


def objectNameMatchesLabel(objectName, label):
    if objectName.lower() == label.lower():
        return True

    return False


def uploadSnapshot(snapshot_file_path):
    global imageServerSocket

    if not imageServerSocket:
        connectToImageServer()

    if not imageServerSocket:
        print('Could not connect to ImageServer')
        return

    fd = open(snapshot_file_path, 'rb')
    imageBytes = fd.read()
    fd.close()

    command = bytearray(struct.pack('>BBBBI', 0x42, 0x42, 0x42, 0x42, len(imageBytes)))
    command.extend(imageBytes)

    sent = 0
    while sent < len(command):
        try:
            n = imageServerSocket.send(command[sent:])
        except socket.error:
            n = 0
            imageServerSocket.close()
            imageServerSocket = None

        if n == 0:
            print('Failed to write to ImageServer')
            return

        sent += n

    return


def connectToImageServer():
    global imageServerSocket

    if imageServerSocket:
        return

    imageServerSocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    imageServerSocket.connect((udp.getBotLabAddr(), config.tcpBotLabImagePort))
