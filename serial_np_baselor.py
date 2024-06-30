from pypylon import pylon

# Get the list of connected devices
tl_factory = pylon.TlFactory.GetInstance()
device_info_list = tl_factory.EnumerateDevices()

# Print the serial numbers of the connected Basler cameras
for device_info in device_info_list:
    print("Serial number:", device_info.GetSerialNumber())
