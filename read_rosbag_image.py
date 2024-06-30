import rosbag
import cv2

#bagfile location
bag_file = 'your_bag_file.bag'
#image topic name in which images published
image_topic = '/images'
#output folder to store images
output_folder = './output_images'

bag = rosbag.Bag(bag_file)
for topic, msg, t in bag.read_messages(topics=[image_topic]):
    img = cv2.cvtColor(cv2.flip(msg.data.reshape(msg.height, msg.width, -1), 0), cv2.COLOR_BGR2RGB)
    image_filename = f'{output_folder}/{t.to_nsec()}.jpg'
    cv2.imwrite(image_filename, img)
bag.close()
