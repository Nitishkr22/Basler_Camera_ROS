
import cv2
import os
import rospy
import numpy as np
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from pypylon import pylon

# Specify the serial number of the camera you want to read from
target_serial_number = "24264462"

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
rospy.init_node('image_publisher', anonymous=True)
# Create a publisher with the appropriate topic and message type
image_pub = rospy.Publisher('images', Image, queue_size=10)
# Create a CvBridge object to convert OpenCV images to ROS messages
bridge = CvBridge()
while camera.IsGrabbing():
    grabResult = camera.RetrieveResult(5000, pylon.TimeoutHandling_ThrowException)
    if grabResult.GrabSucceeded():
        # Access the image data
        image = converter.Convert(grabResult)
        img = image.GetArray()
        frame = cv2.resize(img,(1920,1080),interpolation=cv2.INTER_AREA)
        # Convert the image to a ROS message
        image_msg = bridge.cv2_to_imgmsg(frame, encoding='bgr8')
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
