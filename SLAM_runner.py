import subprocess
import time
import signal



command = "roslaunch realsense2_camera opensource_tracking.launch enable_gyro:=true enable_accel:=true unite_imu_method:=linear_interpolation"
while True:
    process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, shell=True)
    t = time.time()
    stopping = False
    while time.time() - t < 5:
        output = process.stdout.readline()
        if output == '' and process.poll() is not None:
            break
        if output:
            print(output.strip())
    process.kill()
    print("TERMINATED")
    time.sleep(5)
    print("NEW SESSION STARTING")
