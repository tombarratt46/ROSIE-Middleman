from fastapi import FastAPI, Response, responses
from pympler import asizeof
import rospy
from geometry_msgs.msg import Twist, PointStamped
from sensor_msgs.msg import Image
from std_msgs.msg import String, Empty
from nav_msgs.msg import Odometry
import numpy as np
import open3d as o3d
import os
import subprocess
import time

DEBUG = True
print(f"Debug mode: {DEBUG}")

app = FastAPI()

rospy.init_node('API', anonymous=True)

key_pressed = ""
distance = 500000

print("Finished setup")

def depth_cb(data):
    global distance, prev_calc
    print(f"Depth callback called")
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
    #print(f"Distance: {distance}")

@app.get("/scan")
async def scan():
    print(f"[/scan] Endpoint Called")
    pub = rospy.Publisher('scan', Empty, queue_size=1)
    pub.publish(Empty())
    return


@app.get("/goto/{x}/{y}")
async def goto(x,y):
    print(f"[/goto] Endpoint Called - {x}, {y}")
    
    ps = PointStamped()
    ps.header.frame_id = "map"
    ps.point.x = float(x)
    ps.point.y = float(y)
    ps.point.z = 0.0
    #Publish the point
    pub = rospy.Publisher('goto', PointStamped, queue_size=10)
    pub.publish(ps)
    print("Published")

    return #{"Point Published: {x}, {y}"}


@app.get("/telemetry")
async def telemetry():
    if DEBUG:
        return {"x": 0, "y": 0, "z": 0, "roll":0, "pitch":0, "yaw":0}
    else:
        odo_msg = rospy.wait_for_message('/odometry/filtered', Odometry)
        return {"x": odo_msg.pose.pose.position.x, "y": odo_msg.pose.pose.position.y, "z": odo_msg.pose.pose.position.z, "roll": odo_msg.pose.pose.orientation.x, "pitch": odo_msg.pose.pose.orientation.y, "yaw": odo_msg.pose.pose.orientation.z}

@app.get("/status")
async def status():
    try:
        status_msg = rospy.wait_for_message('/robot/status', String, timeout=1)
    except:
        status_msg = String()
        status_msg.data = "Unknown"
    return {"status": status_msg.data}


@app.get("/control/{command}")
async def control(command):
    global key_pressed, distance
    pub = rospy.Publisher('loco', Twist, queue_size=10)
    twist = Twist()
    print(distance)
    if command == "forward" and distance > 500:
        twist.linear.x = 0.2
    elif command == "back":
        twist.linear.x = -0.2
    elif command == "left":
        twist.angular.z = 0.35
    elif command == "right":
        twist.angular.z = -0.35
    elif command == "stop":
        twist.linear.x = 0.0
        twist.angular.z = 0.0

        emergency_pub = rospy.Publisher('emergency', Empty, queue_size=1)
        emergency_pub.publish(Empty())

    #logging.debug(f"[/control] Command: {command}, Twist: {twist}")
    pub.publish(twist)

@app.get("/pointcloud")
async def pointcloud(response: Response): 
    Response.content_type = "application/octet-stream"
    print("Pointcloud endpoint called")
    if not os.path.exists("tmp"):
        print("Creating tmp directory")
        os.mkdir("tmp")

    for f in os.listdir("tmp"):
        os.remove(os.path.join("tmp", f))
        print(f"Removed {f} from tmp directory")
    if DEBUG:
        print(f"[/pointcloud] Debug mode, using pre-recorded pointcloud")
        filename = "debug.pcd"
    else:
        print(f"[/pointcloud] Recording pointcloud")
        process = subprocess.Popen("rosrun pcl_ros pointcloud_to_pcd input:=/rtabmap/cloud_map _prefix:=tmp/pc-", stdout=subprocess.PIPE, stderr=subprocess.STDOUT, shell=True)
        while len(os.listdir("tmp")) == 0:
            pass
        print(f"[/pointcloud] File created, waiting for stopped writing")
        time.sleep(1) # TODO find a better way to wait for the file to be written
        process.kill()
        filename = os.listdir("tmp")[0]
        print(f"[/pointcloud] File written to: {filename}")
    newfilename = filename.split('.')[0] + ".xyzrgb"
    if DEBUG:
        pc = o3d.io.read_point_cloud(f'{filename}')
    else:
        pc = o3d.io.read_point_cloud(f'tmp/{filename}')
    print(f"[/pointcloud] Point cloud loaded, size: {asizeof.asizeof(pc)}")
    clean_pc, _ = pc.remove_statistical_outlier(nb_neighbors=20, std_ratio=1.25)
    print(f"[/pointcloud] Point cloud cleaned, size: {asizeof.asizeof(clean_pc)}")
    o3d.io.write_point_cloud(f'tmp/{newfilename}', clean_pc, write_ascii=True)
    print(f"[/pointcloud] New point cloud written to: {newfilename}")

    return responses.FileResponse(f'tmp/{newfilename}', media_type="text/plain", headers={"Content-Disposition": "attachment"})
