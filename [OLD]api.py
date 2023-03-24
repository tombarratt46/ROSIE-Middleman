import http.server
import socketserver
import rospy
import os
import subprocess
import time
import open3d as o3d

class CustomHandler(http.server.SimpleHTTPRequestHandler):
    def do_GET(self):
        # Set the HTTP response headers
        self.send_response(200)

        if not os.path.exists("tmp"):
            os.mkdir("tmp")

        for f in os.listdir("tmp"):
            print(f)
            os.remove(os.path.join("tmp", f))

        process = subprocess.Popen("rosrun pcl_ros pointcloud_to_pcd input:=/rtabmap/cloud_map _prefix:=tmp/pc-", stdout=subprocess.PIPE, stderr=subprocess.STDOUT, shell=True)
        while len(os.listdir("tmp")) == 0:
            pass
        time.sleep(1) # TODO find a better way to wait for the file to be written
        process.kill()
        filename = os.listdir("tmp")[0]
        newfilename = filename.split('.')[0] + ".xyzrgb"
        pc = o3d.io.read_point_cloud(f'tmp/{filename}')
        o3d.io.write_point_cloud(f'tmp/{newfilename}', pc)

        self.send_header('Content-type', 'application/octet-stream')
        self.send_header('Content-Disposition', f'attachment; filename="{filename}"')
        self.end_headers()
        with open(f'tmp/{newfilename}', 'rb') as f:
            self.wfile.write(f.read())



if __name__ == '__main__':
    rospy.init_node('point_cloud_server')
    PORT = 8080
    Handler = CustomHandler
    with socketserver.TCPServer(("", PORT), Handler) as httpd:
        print("serving at port", PORT)
        httpd.serve_forever()
