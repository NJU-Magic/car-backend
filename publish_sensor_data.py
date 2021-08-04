import rospy
import cv2
from threading import Thread, Event
from flask import Flask, render_template, Response
import signal, sys
#from cv_bridge import CvBridge
from sensor_msgs.msg import Image, Imu, LaserScan, CameraInfo
import numpy as np
import math
from flask_cors import *
import math
import json
# import matplotlib.pyplot as plt
# cmap = plt.cm.inferno
frame_rgb = None
frame_lidar = None
frame_depth = None
imu_data = None
rgb_intrin = None
depth_intrin = None

sensor_data = {
    "rgb_frame" : None,
    "rgb_frame_stamp": None,
    "rgb_frame_seq": None,
    "imu_data": None,
    "imu_data_stamp": None,
    "imu_data_seq": None,
    "depth_raw": None,
    "depth_raw_stamp": None,
    "depth_raw_seq": None,
    "lidar_raw": None,
    "lidar_raw_stamp": None,
    "lidar_raw_seq": None,
    "rgb_intrin": None,
    "depth_intrin": None
}
#bridge = CvBridge()
event_rgb = Event()
event_lidar = Event()
event_depth = Event()
event_imu = Event()
event_rgb_intrin = Event()
event_depth_intrin = Event()

def on_imu(data):
    global imu_data, sensor_data
    x = data.orientation.x
    y = data.orientation.y
    z = data.orientation.z
    w = data.orientation.w

    r = math.atan2(2*(w*x+y*z),1-2*(x*x+y*y))
    f = 2*(w*y-z*z)
    p = 0
    if(-1<=f<=1):
        p = math.asin(f)
    y = math.atan2(2*(w*z+x*y),1-2*(z*z+y*y))

    roll = round(r*180/math.pi, 2)
    pitch = round(p*180/math.pi, 2)
    yaw = round(y*180/math.pi, 2)
    imu_data = {
        "yaw" : yaw,
        "roll" : roll,
        "pitch" : pitch,
        "x":1,
        "y":1,
        "z":1
    }

    sensor_data["imu_data"] = imu_data
    sensor_data["imu_data_stamp"] = data.header.stamp.to_sec()
    sensor_data["imu_data_seq"] = data.header.seq
    event_imu.set()

def on_depth(data):
    global frame_depth

    depth32 = np.frombuffer(data.data, dtype=np.float32)
    depth32 = depth32.reshape((data.height, data.width))
    depth = np.zeros((data.height, data.width))
    depth = depth + depth32
    depth[np.isnan(depth)] = 0
    #np.save("a.npy",depth)
    r = np.max(depth) - np.min(depth)
    if r != 0:
        depth = (depth - np.min(depth)) / r * 255
    else:
        depth = np.zeros((data.height, data.width))
    depth = depth.astype(np.uint8)
    # color_depth = 255*cmap(depth)[:,:,:3]
    color_depth = cv2.applyColorMap(depth,cv2.COLORMAP_JET)
    frame_depth = cv2.imencode(".jpg", color_depth)[1].tobytes()

    sensor_data["depth_raw"] = list(data.data)
    sensor_data["depth_raw_stamp"] = data.header.stamp.to_sec()
    sensor_data["depth_raw_seq"] = data.header.seq
    event_depth.set()

def on_image(data):
    global frame_rgb, sensor_data
    #cv_image = cv2.flip(cv2.flip(bridge.imgmsg_to_cv2(msg, desired_encoding = "passthrough"),0),1)
    #cv_image = cv2.rotate(bridge.imgmsg_to_cv2(msg, desired_encoding = "passthrough"), cv2.ROTATE_90_CLOCKWISE)
    #frame = cv2.imencode(".jpg",cv_image)[1].tobytes()
    h = data.height
    w = data.width
    total_length = data.height * data.step
    #print(data.height, data.width, data.step)
    #rgb = np.zeros(data.height * data.width * 3)
    rgb = np.frombuffer(data.data, dtype=np.uint8)
    #for i in range(total_length):
        #rgb[i] = data.data[i]
    rgb = rgb.reshape((data.height, data.width, 3))[:,:,::-1]
    frame_rgb = cv2.imencode(".jpg",rgb)[1].tobytes()

    #sensor_data["rgb_frame"] = rgb.tolist()
    sensor_data["rgb_frame"] = list(data.data)
    sensor_data["rgb_frame_stamp"] = data.header.stamp.to_sec()
    sensor_data["rgb_frame_seq"] = data.header.seq
    event_rgb.set()

def on_lidar(data):
    global frame_lidar, sensor_data
    img = np.zeros((600, 600,3), np.uint8)
    angle = data.angle_min
    for r in data.ranges:
        if math.isinf(r) == True:
            r = 0
        x = math.trunc((r * 50.0)*math.cos(angle + (-90.0*3.1416/180.0)))
        y = math.trunc((r * 50.0)*math.sin(angle + (-90.0*3.1416/180.0)))

        #set the borders (all values outside the defined area should be 0)
        if y > 600 or y < -600 or x<-600 or x>600:
            x=0
            y=0
        cv2.line(img,(300, 300),(x+300,y+300),(255,0,0),2)
        angle= angle + data.angle_increment 
    cv2.circle(img, (300, 300), 2, (255, 255, 0))
    frame_lidar = cv2.imencode(".jpg",img)[1].tobytes()

    sensor_data["lidar_raw"] = list(data.data)
    sensor_data["lidar_raw_stamp"] = data.header.stamp.to_sec()
    sensor_data["lidar_raw_seq"] = data.header.seq
    event_lidar.set()

def on_rgb_intrin(data):
    global rgb_intrin, sensor_data

    P = np.array(data.P)
    P = P.reshape((3, 4))
    rgb_intrin = P.tolist()
    rgb_intrin.append([0.0, 0.0, 0.0, 1.0])

    sensor_data["rgb_intrin"] = rgb_intrin
    event_rgb_intrin.set()

def on_depth_intrin(data):
    global depth_intrin, sensor_data

    P = np.array(data.P)
    P = P.reshape((3, 4))
    depth_intrin = P.tolist()
    depth_intrin.append([0.0, 0.0, 0.0, 1.0])

    sensor_data["depth_intrin"] = depth_intrin
    event_depth_intrin.set()

#def on_depth_intrin(data):



Thread(target=lambda: rospy.init_node('cam_listener', disable_signals=True)).start()
rospy.Subscriber("/camera/image", Image, on_image)
rospy.Subscriber("/scan", LaserScan, on_lidar)
rospy.Subscriber("/camera/depth/image", Image, on_depth)
rospy.Subscriber("/imu/data", Imu, on_imu)
rospy.Subscriber("/camera/camera_info", CameraInfo, on_rgb_intrin)
#rospy.Subscriber("/camera/depth/camera_info", CameraInfo, on_depth_intrin)

app = Flask(__name__)
CORS(app, supports_credentials=True)

def get_frame():
    event_rgb.wait()
    event_rgb.clear()
    return frame_rgb

def get_frame_lidar():
    event_lidar.wait()
    event_lidar.clear()
    return frame_lidar

def get_frame_depth():
    event_depth.wait()
    event_depth.clear()
    return frame_depth

def get_imu():
    event_imu.wait()
    event_imu.clear
    return imu_data

@app.route('/')
def index():
    return render_template('index.html')

# Ladar
def gen_lidar():
    while True:
        frame_lidar = get_frame_lidar()
        yield (b'--frame\r\n'
                b'Content-Type: image/jpeg\r\n\r\n' + frame_lidar + b'\r\n')
@app.route('/lidar_feed')
def lidar_feed():
    return Response(gen_lidar(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

# RGB
def gen_rgb():
    while True:
        frame = get_frame()
        yield (b'--frame\r\n'
                b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
@app.route('/video_feed')
def video_feed():
    return Response(gen_rgb(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

# Depth
def gen_depth():
    while True:
        frame_depth = get_frame_depth()
        yield (b'--frame\r\n'
                b'Content-Type: image/jpeg\r\n\r\n' + frame_depth + b'\r\n')
@app.route('/depth_feed')
def depth_feed():
    return Response(gen_depth(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

def signal_handler(signal, frame):
    rospy.signal_shutdown("end")
    sys.exit(0)

signal.signal(signal.SIGINT,signal_handler)

@app.route('/connection', methods=['POST'])
@cross_origin()
def connection():
    return {}

# Imu
@app.route('/imu', methods=['POST'])
@cross_origin()
def get_imu():
    global imu_data
    return imu_data

# RGB intrinsic
@app.route('/rgb_intrin', methods=['POST'])
@cross_origin()
def get_rgb_intrin():
    global rgb_intrin
    return rgb_intrin

# Depth intrinsic
@app.route('/depth_intrin', methods=['POST'])
@cross_origin()
def get_depth_intrin():
    global depth_intrin
    return depth_intrin

class MyEncoder(json.JSONEncoder):
    def default(self, obj):
        if isinstance(obj, np.ndarray):
            return obj.tolist()
        elif isinstance(obj, bytes):
            return str(obj, encoding='utf-8');
        return json.JSONEncoder.default(self, obj)


@app.route('/upload_sensor_data', methods=['POST'])
@cross_origin()
def upload_sensor_data():
    global sensor_data
    return sensor_data
    #return json.dumps(sensor_data,cls=MyEncoder,indent=4)

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000 ,debug=True)
