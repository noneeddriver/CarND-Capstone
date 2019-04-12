#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
import tf
import cv2
import yaml
from scipy.spatial import KDTree


STATE_COUNT_THRESHOLD = 3


class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None
        self.whole_waypoints = None
        self.waypoints_2d = None
        self.waypoints_tree = None
        self.waypoints_num = None
        self.camera_image = None
        self.lights = []
        self.has_image = False

        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb)

        config_string = rospy.get_param("/traffic_light_config")
        # self.config = yaml.load(config_string, Loader = yaml.FullLoader) # for PyYAML 5.1 and later
	self.config = yaml.load(config_string)
        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=2)

        self.bridge = CvBridge()
        self.light_classifier = TLClassifier()
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0
        self.stopline_waypoints_idx = []
        
        rospy.spin()

    def pose_cb(self, msg):
        self.pose = [msg.pose.position.x, msg.pose.position.y] 

    def waypoints_cb(self, waypoints):
        self.whole_waypoints = waypoints
        if not self.waypoints_2d:
            self.waypoints_2d = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] \
                                 for waypoint in waypoints.waypoints]
            self.waypoints_num = len(self.waypoints_2d)
            self.waypoints_tree = KDTree(self.waypoints_2d)

    def traffic_cb(self, msg):
        self.lights = msg.lights

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
        self.has_image = True
        self.camera_image = msg
        stopline_wp, state = self.process_traffic_lights()
##        print ("stopline_wp: %d, state: %s" %(stopline_wp, state))
             
        '''
        Publish upcoming red lights at camera frequency.
        Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        of times till we start using it. Otherwise the previous stable state is
        used.
        '''
        if self.state != state:
            self.state_count = 0
            self.state = state
        elif self.state_count >= STATE_COUNT_THRESHOLD:
            self.last_state = self.state
            stopline_wp = stopline_wp if state == TrafficLight.RED else -1
            self.last_wp = stopline_wp
            self.upcoming_red_light_pub.publish(Int32(stopline_wp))
	    prediction_str = "unknown"
        else:
            self.upcoming_red_light_pub.publish(Int32(self.last_wp))
        self.state_count += 1
	if self.state != self.last_state:
	    if self.state == 0:
	        prediction_str = "red"
	    elif self.state == 1 and (self.last_state == 2 or self.last_state == 4):
	        prediction_str = "yellow"
	    elif self.state == 2:
	        prediction_str = "green"
	    else:
	        prediction_str = "unknow"
            print "Traffic Light Detection: ", prediction_str
		

    def get_closest_waypoint(self, pose):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
        #TODO implement
        if pose and self.waypoints_tree:
            return self.waypoints_tree.query(pose)[1]
        else:
            raise Exception,"function get_closest_waypoint called before initialized!"

    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        if(not self.has_image):
            self.prev_light_loc = None
            return False

        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")

        #Get classification
        return self.light_classifier.get_classification(cv_image)

        # just for testing:
        # return light.state

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        light = None
        
        # List of positions that correspond to the line to stop in front of for a given intersection
        self.stop_line_positions = self.config['stop_line_positions']
        if self.pose and self.waypoints_tree and len(self.stopline_waypoints_idx) == 0:
           #  print "stopline: \n"
            for stopline in self.stop_line_positions:
                idx = self.get_closest_waypoint(stopline)
                self.stopline_waypoints_idx.append(idx)
                # print idx,", "
                
        if(self.pose and self.waypoints_tree):
            car_position = self.get_closest_waypoint(self.pose)

        #TODO find the closest visible traffic light (if one exists)
        if self.waypoints_num:
            idx_closest_stopline = 2 * self.waypoints_num
            for idx in self.stopline_waypoints_idx:
                if (car_position > self.waypoints_num - 100) and (idx < 100):
                    idx += self.waypoints_num
                if idx > car_position and idx < car_position + 100:
                    if (idx - car_position) < (idx_closest_stopline - car_position):
                        idx_closest_stopline = idx
            light = self.lights[self.stopline_waypoints_idx.index(idx_closest_stopline)] \
                    if idx_closest_stopline <= self.waypoints_num else None        
            if light:
                state = self.get_light_state(light)
                return idx_closest_stopline, state
            self.waypoints = None
        return -1, TrafficLight.UNKNOWN

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
