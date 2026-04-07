import rospy  
from sensor_msgs.msg import Image   
import cv2
import time
import numpy as np
import PIL
from yolo import YOLO
import sys
import numpy as np
from sensor_msgs.msg import Image

nms = 0.3 # 全局变量 NMS 值

def imgmsg_to_cv2(img_msg):
    if img_msg.encoding != "bgr8":
        rospy.logerr("This Coral detect node has been hardcoded to the 'bgr8' encoding.  Come change the code if you're actually trying to implement a new camera")
    dtype = np.dtype("uint8")
    dtype = dtype.newbyteorder('>' if img_msg.is_bigendian else '<')
    image_opencv = np.ndarray(shape=(img_msg.height, img_msg.width, 3), dtype=dtype, buffer=img_msg.data)
    if img_msg.is_bigendian == (sys.byteorder == 'little'):
        image_opencv = image_opencv.byteswap().newbyteorder()
    return image_opencv

def cv2_to_imgmsg(cv_image):
    img_msg = Image()
    img_msg.height = cv_image.shape[0]
    img_msg.width = cv_image.shape[1]
    img_msg.encoding = "bgr8"
    img_msg.is_bigendian = 0
    img_msg.data = cv_image.tostring()
    img_msg.step = len(img_msg.data) // img_msg.height
    return img_msg

def image_callback(msg):
    global pub, yolo

    frame = imgmsg_to_cv2(msg)
    frame = cv2.cvtColor(frame,cv2.COLOR_BGR2RGB)
    frame = PIL.Image.fromarray(np.uint8(frame))
    frame = np.array(yolo.detect_image(frame, nms))
    frame = cv2.cvtColor(frame,cv2.COLOR_RGB2BGR)
  
    pub.publish(cv2_to_imgmsg(frame))  
  
def main():  
    global pub, yolo
    yolo = YOLO() 
    rospy.init_node('image_listener', anonymous=True)
    pub = rospy.Publisher('/camera/detect_image', Image, queue_size=10)  
    rospy.Subscriber('/camera/image', Image, image_callback)  
    rospy.spin()  
  
if __name__ == '__main__':  
    main()
