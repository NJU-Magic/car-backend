import rospy
import cv2
from threading import Thread, Event
from flask import Flask, render_template, Response
import signal, sys
#from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan # 激光雷达
import numpy as np
import math
from flask_cors import *

# import matplotlib.pyplot as plt
# cmap = plt.cm.inferno
frame_rgb = None
frame_laser = None
frame_depth = None
#bridge = CvBridge()
event_rgb = Event()
event_laser = Event()
event_depth = Event()

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
    event_depth.set()

def on_image(data):
    global frame
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
    event_rgb.set()

def on_laser(data):
    #激光雷达
    global frame_laser
    img = np.zeros((600, 600,3), np.uint8)
    angle = data.angle_min
    for r in data.ranges:
        if math.isinf(r) == True:
            r = 0
        x = math.trunc((r * 50.0)*math.cos(angle + (-90.0*3.1416/180.0)))
        y = math.trunc((r * 50.0)*math.sin(angle + (-90.0*3.1416/180.0)))

        #set the borders (all values outside the defined area should be 0)
        #设置限度，基本上不设置也没关系了
        if y > 600 or y < -600 or x<-600 or x>600:
            x=0
            y=0
        cv2.line(img,(300, 300),(x+300,y+300),(255,0,0),2)
        angle= angle + data.angle_increment 
    cv2.circle(img, (300, 300), 2, (255, 255, 0))
    frame_laser = cv2.imencode(".jpg",img)[1].tobytes()
    event_laser.set()


Thread(target=lambda: rospy.init_node('cam_listener', disable_signals=True)).start()
rospy.Subscriber("/camera/image", Image, on_image)
rospy.Subscriber("/lidar/scan", LaserScan, on_laser) #激光雷达
rospy.Subscriber("/camera/depth/image", Image, on_depth)
#深度图

app = Flask(__name__)
CORS(app, supports_credentials=True)

def get_frame():
    event_rgb.wait()
    event_rgb.clear()
    return frame_rgb

def get_frame_laser():
    event_laser.wait()
    event_laser.clear()
    return frame_laser

def get_frame_depth():
    event_depth.wait()
    event_depth.clear()
    return frame_depth

@app.route('/')
def index():
    return render_template('index.html')



def gen_laser():
    while True:
        frame_laser = get_frame_laser()
        yield (b'--frame\r\n'
                b'Content-Type: image/jpeg\r\n\r\n' + frame_laser + b'\r\n')


@app.route('/laser_feed')
def laser_feed():
    return Response(gen_laser(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')


def gen_rgb():
    while True:
        frame = get_frame()
        yield (b'--frame\r\n'
                b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

@app.route('/video_feed')
def video_feed():
    return Response(gen_rgb(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

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


if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000 ,debug=True)
