import cv2
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

# Set camera parameters
camera.Open()
camera.Width.SetValue(1280)
camera.Height.SetValue(720)

# Start grabbing images
camera.StartGrabbing(pylon.GrabStrategy_LatestImageOnly)

# Create a window to display the camera feed
cv2.namedWindow("Camera Feed", cv2.WINDOW_NORMAL)

# Process images
while camera.IsGrabbing():
    grab_result = camera.RetrieveResult(5000, pylon.TimeoutHandling_ThrowException)

    if grab_result.GrabSucceeded():
        # Access the image data
        image = grab_result.Array

        # Convert the image to BGR format
        bgr_image = cv2.cvtColor(image, cv2.COLOR_GRAY2RGB)

        # Display the image in the window
        cv2.imshow("Camera Feed", bgr_image)

    grab_result.Release()

    # Check for key press to exit
    if cv2.waitKey(1) == 27:
        break

# Stop grabbing images, close the camera, and destroy the window
camera.StopGrabbing()
camera.Close()
cv2.destroyAllWindows()
