
import cv2
import os
import rospy
import numpy as np
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from pypylon import pylon



def cv2_to_imgmsg(cv_image):
    img_msg = Image()
    img_msg.height = cv_image.shape[0]
    img_msg.width = cv_image.shape[1]
    img_msg.encoding = "bgr8"
    img_msg.is_bigendian = 0
    img_msg.data = cv_image.tostring()
    img_msg.step = len(img_msg.data) // img_msg.height # That double line is actually integer division, not a comment
    return img_msg

# Specify the serial number of the camera you want to read from
target_serial_number = "24264461"

# Connect to the specified camera
tl_factory = pylon.TlFactory.GetInstance()
device_info_list = tl_factory.EnumerateDevices()
for device_info in device_info_list:
    if device_info.GetSerialNumber() == target_serial_number:
        camera = pylon.InstantCamera(tl_factory.CreateDevice(device_info))
        break
else:
    raise ValueError(f"Camera with serial number {target_serial_number} not found.")

camera.StartGrabbing(pylon.GrabStrategy_LatestImageOnly) 
converter = pylon.ImageFormatConverter()
converter.OutputPixelFormat = pylon.PixelType_BGR8packed
converter.OutputBitAlignment = pylon.OutputBitAlignment_MsbAligned



 # Initialize the ROS node
rospy.init_node('image_publisher1', anonymous=True)
# Create a publisher with the appropriate topic and message type
image_pub = rospy.Publisher('camera1_images', Image, queue_size=10)
# Create a CvBridge object to convert OpenCV images to ROS messages
bridge = CvBridge()
while camera.IsGrabbing():
    grabResult = camera.RetrieveResult(5000, pylon.TimeoutHandling_ThrowException)
    if grabResult.GrabSucceeded():
        # Access the image data
        image = converter.Convert(grabResult)
        img = image.GetArray()
        frame = cv2.resize(img,(1280,720),interpolation=cv2.INTER_AREA)
        # Convert the image to a ROS message
        image_msg = cv2_to_imgmsg(frame)
        # Publish the image message
        image_pub.publish(image_msg)
        cv2.imshow('Camera', frame)
        # Exit the loop if the 'q' key is pressed
        if cv2.waitKey(1) == ord('q'):
            break

    grabResult.Release()

# Releasing the resource    
camera.StopGrabbing()
cv2.destroyAllWindows()
