#!/home/arjun/anaconda3/bin/python
##
#
# Control a MAV via mavros
#
##

# python opencv_object_tracking.py --video dashcam_boston.mp4 --tracker csrt

import rospy
import tf
from geometry_msgs.msg import Pose, PoseStamped, Twist, Quaternion
from mavros_msgs.msg import OverrideRCIn
from mavros_msgs.msg import RCIn
from mavros_msgs.srv import CommandBool
from mavros_msgs.srv import SetMode
from mavros_msgs.srv import CommandTOL
from imutils.video import VideoStream
from imutils.video import FPS
import argparse
import imutils
import time
import cv2
import numpy as np

pi_2 = 3.141592654 / 2.0

class MavController:
    """
    A simple object to help interface with mavros
    """
    def __init__(self):

        rospy.init_node("mav_control_node")
        rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.pose_callback)
        rospy.Subscriber("/mavros/rc/in", RCIn, self.rc_callback)

        self.cmd_pos_pub = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size=1)
        self.cmd_vel_pub = rospy.Publisher("/mavros/setpoint_velocity/cmd_vel_unstamped", Twist, queue_size=1)
        self.rc_override = rospy.Publisher("/mavros/rc/override", OverrideRCIn, queue_size=1)

        # mode 0 = STABILIZE
        # mode 4 = GUIDED
        # mode 9 = LAND
        self.mode_service = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        self.arm_service = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.takeoff_service = rospy.ServiceProxy('/mavros/cmd/takeoff', CommandTOL)

        self.rc = RCIn()
        self.pose = Pose()
        self.timestamp = rospy.Time()

    def rc_callback(self, data):
        """
        Keep track of the current manual RC values
        """
        self.rc = data

    def pose_callback(self, data):
        """
        Handle local position information
        """
        self.timestamp = data.header.stamp
        self.pose = data.pose

    def goto(self, pose):
        """
        Set the given pose as a the next setpoint by sending
        a SET_POSITION_TARGET_LOCAL_NED message. The copter must
        be in GUIDED mode for this to work.
        """
        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = self.timestamp
        pose_stamped.pose = pose

        self.cmd_pos_pub.publish(pose_stamped)

    def goto_xyz_rpy(self, x, y, z, ro, pi, ya):
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z

        quat = tf.transformations.quaternion_from_euler(ro, pi, ya + pi_2)

        pose.orientation.x = quat[0]
        pose.orientation.y = quat[1]
        pose.orientation.z = quat[2]
        pose.orientation.w = quat[3]
        self.goto(pose)
        #print(quat)

    def set_vel(self, vx, vy, vz, avx=0, avy=0, avz=0):
        """
        Send comand velocities. Must be in GUIDED mode. Assumes angular
        velocities are zero by default.
        """
        cmd_vel = Twist()

        cmd_vel.linear.x = vx
        cmd_vel.linear.y = vy
        cmd_vel.linear.z = vz

        cmd_vel.angular.x = avx
        cmd_vel.angular.y = avy
        cmd_vel.angular.z = avz

        self.cmd_vel_pub.publish(cmd_vel)

    def arm(self):
        """
        Arm the throttle
        """
        return self.arm_service(True)

    def disarm(self):
        """
        Disarm the throttle
        """
        return self.arm_service(False)

    def takeoff(self, height=1.0):
        """
        Arm the throttle, takeoff to a few feet, and set to guided mode
        """
        # Set to stabilize mode for arming
        #mode_resp = self.mode_service(custom_mode="0")
        mode_resp = self.mode_service(custom_mode="4")
        self.arm()

        # Set to guided mode
        #mode_resp = self.mode_service(custom_mode="4")

        # Takeoff
        takeoff_resp = self.takeoff_service(altitude=height)

        #return takeoff_resp
        return mode_resp

    def land(self):
        """
        Set in LAND mode, which should cause the UAV to descend directly,
        land, and disarm.
        """
        resp = self.mode_service(custom_mode="9")
        self.disarm()

def simple_demo():
    """
    A simple demonstration of using mavros commands to control a UAV.
    """
    c = MavController()
    rospy.sleep(1)

    print("Takeoff")
    c.takeoff(0.5)
    rospy.sleep(3)
    c.goto_xyz_rpy(0,0,5,0,0,0)
    rospy.sleep(10)

    #print("Waypoint 1: position control")
    #c.goto_xyz_rpy(0.4,0.0,1.2,0,0,0)
    #rospy.sleep(4)
    #print("Waypoint 2: position control")
    #c.goto_xyz_rpy(0.4,0.4,1.2,0,0,0)
    #rospy.sleep(4)
    #print("Waypoint 3: position control")
    #c.goto_xyz_rpy(0.0,0.4,1.2,0,0,0)
    #rospy.sleep(4)
    #print("Waypoint 4: position control")
    #c.goto_xyz_rpy(0,0,1.2,0,0,0)
    #rospy.sleep(4)

    #print("Velocity Setpoint 1")
    #c.set_vel(0,0.1,0)
    #rospy.sleep(5)
    #print("Velocity Setpoint 2")
    #c.set_vel(0,-0.1,0)
    #rospy.sleep(5)
    #print("Velocity Setpoint 3")
    #c.set_vel(0,0,0)
    #rospy.sleep(5)

    print("Landing")
    c.land()

def control_visual(center,frame, frameX, frameY, divs = 4):
	#lenX = frameX//math.sqrt(divs)
	#lenY = frameY//math.sqrt(divs)

	lenX = frameX//2
	lenY = frameY//2
	
	leftUpper = frame[0:lenX, 0:lenY]
	rightUpper = frame[lenX+1:frameX, 0:lenY]
	leftLower = frame[0:lenX, lenY+1: frameY]
	rightLower = frame[lenX:frameX, lenY+1: frameY]

	if (center[0] in range(0,lenX)):
		if (center[1] in range(0,lenY)):
			c.goto_xyz_rpy(2,-2,7,0,0,0)
			rospy.sleep(0.3)
        		#print("[Region 1]")
		elif (center[1] in range(lenY+1, frameY)):
			c.goto_xyz_rpy(2,5,3,0,0,0)
			rospy.sleep(0.3)
        		#print("[Region 3]")
	
	if (center[0] in range(lenX+1,frameX)): 
		if( center[1] in range(0,lenY)):
			c.goto_xyz_rpy(5,-2,7,0,0,0)
			rospy.sleep(.3)
        		#print("[Region 2]")
		elif (center[1] in range(lenY+1, frameY)):
			c.goto_xyz_rpy(5,5,3,0,0,0)
			rospy.sleep(0.3)
        		#print("[Region 4]")

	'''elif (center[0]   in range(0,lenX) &  center[1] in range(lenY+1, frameY)):
		c.goto_xyz_rpy(5,5,7,0,0,0)
		rospy.sleep(10)
        	print("[Region 3]")

	elif (center[0]   in range(lenX+1,frameX) & center[1]  in range(lenY+1, frameY)):
		c.goto_xyz_rpy(-5,-5,7,0,0,0)
		rospy.sleep(10)
        	print("[Region 4]")'''
    





# construct the argument parser and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-v", "--video", type=str,
	help="path to input video file")
ap.add_argument("-t", "--tracker", type=str, default="kcf",
	help="OpenCV object tracker type")
args = vars(ap.parse_args())

# extract the OpenCV version info
(major, minor) = cv2.__version__.split(".")[:2]

# if we are using OpenCV 3.2 OR BEFORE, we can use a special factory
# function to create our object tracker
if int(major) == 3 and int(minor) < 3:
	tracker = cv2.Tracker_create(args["tracker"].upper())

# otherwise, for OpenCV 3.3 OR NEWER, we need to explicity call the
# approrpiate object tracker constructor:
else:
	# initialize a dictionary that maps strings to their corresponding
	# OpenCV object tracker implementations
	OPENCV_OBJECT_TRACKERS = {
		"csrt": cv2.TrackerCSRT_create,
		"kcf": cv2.TrackerKCF_create,
		"boosting": cv2.TrackerBoosting_create,
		"mil": cv2.TrackerMIL_create,
		"tld": cv2.TrackerTLD_create,
		"medianflow": cv2.TrackerMedianFlow_create,
		"mosse": cv2.TrackerMOSSE_create
	}

	# grab the appropriate object tracker using our dictionary of
	# OpenCV object tracker objects
	tracker = OPENCV_OBJECT_TRACKERS[args["tracker"]]()

# initialize the bounding box coordinates of the object we are going
# to track
initBB = None

# if a video path was not supplied, grab the reference to the web cam
if not args.get("video", False):
	print("[INFO] starting video stream...")
	vs = VideoStream(src=0).start()
	time.sleep(1.0)

# otherwise, grab a reference to the video file
else:
	vs = cv2.VideoCapture(args["video"])

# initialize the FPS throughput estimator
fps = None

#declare a controller 
c = MavController()
rospy.sleep(1)

#takeoff
print("[INFO]Commencing Takeoff")
c.takeoff(5)
rospy.sleep(5)

# loop over frames from the video stream
while True:
	# grab the current frame, then handle if we are using a
	# VideoStream or VideoCapture object
	frame = vs.read()
	frame = frame[1] if args.get("video", False) else frame

	# check to see if we have reached the end of the stream
	if frame is None:
		break

	# resize the frame (so we can process it faster) and grab the
	# frame dimensions
	frame = imutils.resize(frame, width=500)
	(H, W) = frame.shape[:2]

	# check to see if we are currently tracking an object
	if initBB is not None:
		# grab the new bounding box coordinates of the object
		(success, box) = tracker.update(frame)

		# check to see if the tracking was a success
		if success:
			(x, y, w, h) = [int(v) for v in box]
            		cv2.rectangle(frame, (x, y), (x + w, y + h),(0, 255, 0), 2)
            		#print(np.array([x+w//2, y+h//2]))
                    	control_visual(np.array([x+w//2, y+h//2]), frame, W, H, 4)

		# update the FPS counter
		fps.update()
		fps.stop()

		# initialize the set of information we'll be displaying on
		# the frame
		info = [
			("Tracker", args["tracker"]),
			("Success", "Yes" if success else "No"),
			("FPS", "{:.2f}".format(fps.fps())),
		]

		# loop over the info tuples and draw them on our frame
		for (i, (k, v)) in enumerate(info):
			text = "{}: {}".format(k, v)
			cv2.putText(frame, text, (10, H - ((i * 20) + 20)),
				cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
		
		
       

	# show the output frame
	cv2.imshow("Frame", frame)
	key = cv2.waitKey(1) & 0xFF

	# if the 's' key is selected, we are going to "select" a bounding
	# box to track
	if key == ord("s"):
		# select the bounding box of the object we want to track (make
		# sure you press ENTER or SPACE after selecting the ROI)
		initBB = cv2.selectROI("Frame", frame, fromCenter=False,
			showCrosshair=False)

		# start OpenCV object tracker using the supplied bounding box
		# coordinates, then start the FPS throughput estimator as well
		tracker.init(frame, initBB)
		fps = FPS().start()

	# if the `q` key was pressed, break from the loop
	elif key == ord("q"):
		break

# if we are using a webcam, release the pointer
if not args.get("video", False):
	vs.stop()

# otherwise, release the file pointer
else:
	vs.release()

# close all windows
cv2.destroyAllWindows()
