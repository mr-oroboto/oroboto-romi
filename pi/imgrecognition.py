import time
import boto3
import config
import udp

from PIL import Image
from picamera import PiCamera

camera = None
rekognitionClient = None

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
        udp.uploadSnapshot(image_name_cropped)

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