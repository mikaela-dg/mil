import rospy
import numpy as np
from sensor_msgs.msg import Image
from mil_msgs.srv import Enable, EnableResponse
from mil_msgs.srv import Disable, DisableResponse
import cv2
from cv_bridge import CvBridge
from matplotlib.backends.backend_agg import FigureCanvasAgg
from matplotlib.figure import Figure
import matplotlib
import threading

class MilRosPlotter:

    def __init__(self, topic_name, w=20, h=20, dpi=200):
        matplotlib.rcParams.update({'font.size': 22})
        self.pub = rospy.Publisher(topic_name, Image, queue_size=1)
        self.bridge = CvBridge()
        self.fig = Figure(figsize=(w,h), dpi=dpi)
        self.canvas = FigureCanvasAgg(self.fig)
        self.enabled = False
        self.thread = None

        rospy.Service(('/enable_%s'%topic_name), Enable, self.enable)
        rospy.Service(('/disable_%s'%topic_name), Enable, self.disable)


    def enable(self, req):
        self.enabled = True
        return EnableResponse(success=True)


    def disable(self, req):
        self.enabled = False


    def publish_plots(self, plots):
        if not self.enabled:
            return
        if (self.thread is not None) and self.thread.is_alive():
            return
        self.thread = threading.Thread(target=self.publish_plots_, args=(plots,))
        self.thread.daemon = True
        self.thread.start()


    def publish_plots_(self, plots):

        num_of_plots = plots.shape[0]/2

        for i in xrange(1, num_of_plots+1):
            self.fig.add_subplot(num_of_plots, 1, i)
        for i, ax in enumerate(self.fig.axes):
            ax.plot(plots[i*2,:], plots[i*2+1,:])
        self.canvas.draw()

        s, (w, h) = self.canvas.print_to_buffer()

        img = np.fromstring(s, np.uint8).reshape(w, h, 4)

        img = np.roll(img, 3, axis = 2)
        for ax in self.fig.axes: ax.cla()

        # make ros msg of the img
        msg = self.bridge.cv2_to_imgmsg(img, encoding='passthrough')
        # publish the image
        self.pub.publish(msg)

