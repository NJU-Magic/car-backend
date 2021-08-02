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
# import matplotlib.pyplot as plt
# cmap = plt.cm.inferno
frame = None
frame_laser = None
frame_depth = None
#bridge = CvBridge()

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

Thread(target=lambda: rospy.init_node('cam_listener', disable_signals=True)).start()
rospy.Subscriber("/camera/depth/image", Image, on_depth) #深度图

app = Flask(__name__)



    
def get_frame_depth():
    event_depth.wait()
    event_depth.clear()
    return frame_depth

@app.route('/')
def index():
    return render_template('index.html')



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

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5001 ,debug=True)
