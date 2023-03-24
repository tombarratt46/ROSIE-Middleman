from fastAPI import FastAPI


from pynput import keyboard
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from std_msgs.msg import String
import numpy as np

key_pressed = ""
distance = 0
pub = None
cmd_pub = None
def depth_cb(data):
    global distance, prev_calc
    # # if time.time() - prev_calc < 0.5:
    # #     return
    # prev_calc = time.time()
    np_arr = np.fromstring(data.data, np.uint16)
    np_arr = np_arr.reshape(data.height, data.width)
    np_arr[:190,:] = 0
    np_arr[280:,:] = 0
    np_arr[:,:400] = 0
    np_arr[:,448:] = 0
    np_arr[np_arr > 1000] = 0
    np_arr[np_arr < 100] = 0
    np_arr[0,0] = 2000
    distance = np_arr[np_arr!=0].min()


def on_press(key):
    global key_pressed
    key_pressed = key
    #print(key_pressed)


def on_release(key):
    global key_pressed
    key_pressed = ""


def main():
    global pub, cmd_pub
    pub = rospy.Publisher('loco', Twist, queue_size=10)
    cmd_pub = rospy.Publisher('cmd', String, queue_size=10)
    rospy.init_node('loco', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    rospy.Subscriber("camera/depth/image_rect_raw", Image, depth_cb)
    rospy.Timer(rospy.Duration(0.1), key_press)
    rospy.spin()


def key_press(event):
    global key_pressed, distance, pub, cmd_pub
    # print("key pressed: ", key_pressed, "distance: ", distance)
    twist = Twist()
    if key_pressed == keyboard.Key.esc:
        msg = String()
        msg.data = "spin"
        cmd_pub.publish(msg)
        print("SPIN")
    print(distance)
    if key_pressed == keyboard.Key.up and distance > 500:
        twist.linear.x = 0.3
    elif key_pressed == keyboard.Key.down:
        twist.linear.x = -0.3
    elif key_pressed == keyboard.Key.left:
        twist.angular.z = 0.4
    elif key_pressed == keyboard.Key.right:
        twist.angular.z = -0.4
    pub.publish(twist)



if __name__ == '__main__':
    listener = keyboard.Listener(on_press=on_press, on_release=on_release)
    listener.start()
    try:
        main()
    except rospy.ROSInterruptException:
        pass
