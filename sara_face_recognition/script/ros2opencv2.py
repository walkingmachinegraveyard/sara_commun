import rospy
import cv2
import cv2.cv as cv
import sys
from std_msgs.msg import String
from sensor_msgs.msg import Image, RegionOfInterest, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import time
import numpy as np


class ROS2OpenCV2(object):
    def __init__(self, node_name):
        self.node_name = node_name

        rospy.init_node(node_name)
        rospy.loginfo("Starting node " + str(node_name))

        rospy.on_shutdown(self.cleanup)

        # parameters to show on screen
        self.show_text = rospy.get_param("~show_text", True)
        self.show_features = rospy.get_param("~show features", True)
        self.show_boxes = rospy.get_param("~show_boxes", True)
        self.flip_image = rospy.get_param("~flip_image", False)
        self.feature_size = rospy.get_param("~feature_size", 1)

        # Initialize the Region of Interest and its publisher
        self.ROI = RegionOfInterest()
        self.roi_pub = rospy.Publisher("/roi", RegionOfInterest, queue_size=1)

        # Initialize a number of global variable
        self.frame = None
        self.frame_size = None
        self.frame_width = None
        self.frame_height = None
        self.depth_image = None
        self.marker_image = None
        self.display_image = None
        self.grey = None
        self.prev_grey = None
        self.selected_point = None
        self.selection = None
        self.drag_start = None
        self.keystroke = None
        self.detect_box = None
        self.track_box = None
        self.display_box = None
        self.keep_marker_history = False
        self.night_mode = False
        self.auto_face_tracking = False
        self.cps = 0
        self.cps_values = list()
        self.cps_n_values = 20
        self.busy = False
        self.resize_window_width = 0
        self.resize_window_height = 0
        self.face_tracking = False

        # Create the main display windows
        self.cv_window_name = self.node_name
        cv.NamedWindow(self.cv_window_name, cv.CV_WINDOW_NORMAL)
        if self.resize_window_height > 0 and self.resize_window_width > 0:
            cv.ResizeWindow(self.cv_window_name, self.resize_window_width, self.resize_window_height)

        # Create the cv_bridge object
        self.bridge = CvBridge()

        # Set a call back on mouse clicks on the image window
        cv.SetMouseCallback(self.node_name, self.on_mouse_click, None)

        # Subcribe to the image and depth topics
        self.image_sub = rospy.Subscriber("input_rgb_image", Image, self.image_callback, queue_size=1)
        self.depth_sub = rospy.Subscriber("input_depth_image", Image, self.depth_callback, queue_size=1)

    def on_mouse_click(self, event, x, y, flags, param):
        if self.frame is None:
            return

        if event == cv.CV_EVENT_LBUTTONUP and not self.drag_start:
            self.features = []
            self.track_box = None
            self.detect_box = None
            self.selected_point = None
            self.drag_start = (x, y)

        if event == cv.CV_EVENT_LBUTTONUP:
            self.drag_start = None
            self.classifier_initialized = False
            self.detect_box = self.selection

        if self.drag_start:
            xmin = max(0, min(x, self.drag_start[0]))
            ymin = max(0, min(y, self.drag_start[1]))
            xmax = min(self.frame_width, max(x, self.drag_start[0]))
            ymax = min(self.frame_height, max(y, self.drag_start[1]))

            self.selection = (xmin, ymin, xmax - xmin, ymax - ymin)

    def image_callback(self, data):
        # Store the image header in a global variable
        self.image_header = data.header

        # Time this loop to get cycles per second
        start = time.time()

        # Convert the ROS image to OpenCV format
        frame = self.convert_image(data)

        # Some webcams invert the image
        if self.flip_image:
            frame = cv2.flip(frame, 0)

        # Store the frame width and height in a pair of global variable
        if self.frame_width is None:
            self.frame_size = (frame.shape[1], frame.shape[0])
            self.frame_width, self.frame_height = self.frame_size

        # Create the marker image we will use for display purposes
        if self.marker_image is None:
            self.marker_image = np.zeros_like(frame)

        # Copy the current frame to the global image
        self.frame = frame.copy

        # Reset the marker image
        if not self.keep_marker_history:
            self.marker_image = np.zeros_like(self.marker_image)

        # Process the image to detect and track objects or features
        processed_image = self.process_image(frame)

        self.processed_image = processed_image.copy()

        # Display the user-selection rectangle or point
        self.display_selection

        # Night mode: only display the markers
        if self.night_mode:
            self.processed_image = np.zeros_like(self.processed_image)

        # Merge the processed_image and the image marker
        self.display_image = cv2.bitwise_or(self.processed_image, self.marker_image)

        # Display the track box
        if self.show_boxes:
            if self.track_box is not None and self.is_rect_nonzero(self.track_box):
                if len(self.track_box) == 4:
                    x, y, w, h = self.track_box
                    size = (w, h)
                    center = (x + w / 2, y + h /2)
                    angle = 0
                    self.track_box = (center, size, angle)
                else:
                    (center, size, angle) = self.track_box

                if self.face_tracking:
                    pt1 = (int(center[0] - size [0] / 2), int(center[1] - size[1] / 2))
                    pt2 = (int(center[0] + size [0] / 2), int(center[1] - size[1] / 2))
                    cv2.rectangle(self.display_image, pt1, pt2, cv.RGB(50, 255, 50), self.feature_size, 8, 0)
                else:
                    vertices = np.int0(cv2.cv.BoxPoints(self.track_box))
                    cv2.drawContours(self.display_image, [vertices], 0, cv.RGB(50, 255, 50), self.feature_size)
            elif self.detect_box is not None and self.is_rect_nonzero(self.detect_box):
                (pt1_x, pt1_y, w, h) = self.detect_box
                if self.show_boxes:
                    cv2.rectangle(self.display_image, (pt1_x, pt1_y), (pt1_x + w, pt1_y + h), cv.RGB(50, 255, 50), self.feature_size, 8, 0)
        # Publish the ROI
        self.publish_ROI()

        # Compute the time for this loop and estimate CPS as a running average
        end = time.time()
        duration = end - start
        fps = int(1.0 / duration)
        self.cps_values.append(fps)
        if len(self.cps_values) > self.cps_n_values:
            self.cps_values.pop(0)
        self.cps = int(sum(self.cps_values) / len(self.cps_values))

        # Display CPS and image resolution if asked to
        if self.show_text:
            font_face = cv2.FONT_HERSHEY_SIMPLEX
            font_scale = 0.5

            if self.frame_size[0] >= 640:
                vstart = 25
                voffset = int(50 + self.frame_size[1] / 120.)
            elif self.frame_size[0] == 320:
                vstart = 15
                voffset = int(35 + self.frame_size[1] / 120.)

            else:
                vstart = 10
                voffset = (int(20 + self.frame_size[1] / 120.))

            cv2.putText(self.display_image, "CPS: " + str(self.cps), (10, vstart), font_face, font_scale, cv.RGB(255, 255, 0))
            cv2.putText(Self.display_image, "RES: " + str(self.frame_size[0]) + "X" + str(self.frame_size[1]), (10, voffset), font_face, font_scale, cv.RGB(255, 255, 0))

        # Update the image display
        cv2.imshow(self.node_name, self.display_image)

        # Process any keyboard command
        self.keystroke = cv2.waitKey(5)
        if self.keystroke is not None and self.keystroke != -1:
            try:
                cc = chr(self.keystroke & 255).lower()
                if cc == 'n':
                    self.night_mode = not self.night_mode
                elif cc == 'f':
                    self.show_features = not self.show_features
                elif cc == 'b':
                    self.show_boxes = not self.show_boxes
                elif cc == 't':
                    self.show_text = not self.show_text
                elif cc == 'q':
                    # The user has press the q key, so exit
                    rospy.signal_shutdown("User hit q key to quit")
            except:
                pass
    def depth_callback(self, data):
        # Convert the ROS image to OpenCV format
        depth_image = self.convert_depth_image(data)

        # Some webcams invert the image
        if self.flip_image:
            depth_image = cv2.flip(depth_image, 0)

        # Process the depth image
        processed_depth_image = self.processed_depth_image(depth_image)

        # Make a global copies
        self.depth_image = depth_image.copy()
        self.processed_depth_image = processed_depth_image.copy()

    def convert_image(self, ros_image):
        # Use cv_bridge to convert the ROS image to OpenCV format
        try:
            cv_image = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
            return np.array(cv_image, dtype=np.uint8)
        except CvBridgeError, e:
            print e

    def convert_depth_image(self, ros_image):
        # Use cv_bridge to convert the ROS image to OpenCV format
        try:
            depth_image = self.bridge.imgmsg_to_cv2(ros_image, "passthrough")

            # Convert to a numpy array since this is what OpenCV uses
            depth_image = np.array(depth_image, dtype=np.float32)

            return depth_image

        except CvBridgeError e:
            print e

    def publish_roi(self):
        if not self.drag_start:
            if self.track_box is not None:
                roi_box = self.track_box
            elif self.detect_box is not None:
                roi_box = self.detect_box
            else:
                return
        try:
            roi_box = self.cvBox2D_to_cvRect(roi_box)

        except:
            return
