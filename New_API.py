from fastapi import FastAPI, Response, responses
from pympler import asizeof
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from std_msgs.msg import String
from nav_msgs.msg import Odometry
import numpy as np
import random
import open3d as o3d
import os
import subprocess
import time
import io
import logging

DEBUG = False
logging.debug(f"Debug mode: {DEBUG}")

app = FastAPI()

rospy.init_node('loco', anonymous=True)

key_pressed = ""
distance = 500000

def depth_cb(data):
    global distance, prev_calc
    logging.debug(f"Depth callback called")
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
    logging.debug(f"Distance: {distance}")


@app.get("/telemetry")
async def telemetry():
    logging.debug(f"[/telemetry] Endpoint Called")
    odo_msg = rospy.wait_for_message('/odometry/filtered', Odometry)
    return {"x": odo_msg.pose.pose.position.x, "y": odo_msg.pose.pose.position.y, "z": odo_msg.pose.pose.position.z, "roll": odo_msg.pose.pose.orientation.x, "pitch": odo_msg.pose.pose.orientation.y, "yaw": odo_msg.pose.pose.orientation.z}


@app.get("/control/{command}")
async def control(command):
    global key_pressed, distance
    pub = rospy.Publisher('loco', Twist, queue_size=10)
    twist = Twist()
    print(distance)
    if command == "forward" and distance > 500:
        twist.linear.x = 0.3
    elif command == "back":
        twist.linear.x = -0.3
    elif command == "left":
        twist.angular.z = 0.4
    elif command == "right":
        twist.angular.z = -0.4
    elif command == "stop":
        twist.linear.x = 0
        twist.angular.z = 0
    logging.debug(f"[/control] Command: {command}, Twist: {twist}")
    pub.publish(twist)

@app.get("/pointcloud")
async def pointcloud(response: Response): 
    # global j
    # return j
    Response.content_type = "application/octet-stream"

    if not os.path.exists("tmp"):
        logging.debug(f"[/pointcloud] Creating tmp directory")
        os.mkdir("tmp")

    for f in os.listdir("tmp"):
        os.remove(os.path.join("tmp", f))
        logging.debug(f"[/pointcloud] Deleting Temp File: {f}")
    if DEBUG:
        logging.debug(f"[/pointcloud] Debug mode, using pre-recorded pointcloud")
        filename = "input.pcd"
    else:
        logging.debug(f"[/pointcloud] Recording pointcloud")
        process = subprocess.Popen("rosrun pcl_ros pointcloud_to_pcd input:=/rtabmap/cloud_map _prefix:=tmp/pc-", stdout=subprocess.PIPE, stderr=subprocess.STDOUT, shell=True)
        while len(os.listdir("tmp")) == 0:
            pass
        logging.debug(f"[/pointcloud] File created, waiting for stopped writing")
        time.sleep(1) # TODO find a better way to wait for the file to be written
        process.kill()
        filename = os.listdir("tmp")[0]
        logging.debug(f"[/pointcloud] File written to: {filename}")
    newfilename = filename.split('.')[0] + ".xyzrgb"
    if DEBUG:
        pc = o3d.io.read_point_cloud(f'{filename}')
    else:
        pc = o3d.io.read_point_cloud(f'tmp/{filename}')
    logging.debug(f"[/pointcloud] Point cloud loaded, size: {asizeof.asizeof(pc)}")
    clean_pc, _ = pc.remove_statistical_outlier(nb_neighbors=20, std_ratio=1.25)
    logging.debug(f"[/pointcloud] Point cloud cleaned, size: {asizeof.asizeof(clean_pc)}")
    o3d.io.write_point_cloud(f'tmp/{newfilename}', clean_pc, write_ascii=True)
    logging.debug(f"[/pointcloud] New point cloud written to: {newfilename}")

    return responses.FileResponse(f'tmp/{newfilename}', media_type="text/plain", headers={"Content-Disposition": "attachment"})

# TODO: Emergency collision detection
# cmd_pub = rospy.Publisher('cmd', String, queue_size=10)

# rate = rospy.Rate(10) # 10hz
# rospy.Subscriber("camera/depth/image_rect_raw", Image, depth_cb)
