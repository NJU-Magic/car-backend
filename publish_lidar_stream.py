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

event_laser = Event()


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

rospy.Subscriber("/lidar/scan", LaserScan, on_laser) #激光雷达
#深度图

app = Flask(__name__)


def get_frame_laser():
    event_laser.wait()
    event_laser.clear()
    return frame_laser
    

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


def signal_handler(signal, frame):
    rospy.signal_shutdown("end")
    sys.exit(0)

signal.signal(signal.SIGINT,signal_handler)

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5002 ,debug=True)
