#!/usr/bin/env python2

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32
from scipy.spatial import KDTree
import numpy as np
import math

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 150 # Number of waypoints we will publish. You can change this number
DECEL_KOMPORT = .5 # komfortable deceleration to the stopline

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /traffic_waypoint below
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.pose = None
        self.whole_waypoints = None
        self.whole_waypoints_2d = None
        self.whole_waypoints_tree = None
        self.lookahead_waypoints = None
        self.stopline_waypoints_idx = -1

        self.loop()
        
    def loop(self):
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            if self.pose and self.whole_waypoints_tree:
                closest_waypoints_idx = self.get_closed_waypoint_idx()
                self.publish_waypoints(closest_waypoints_idx)
            rate.sleep()
            
    def get_closed_waypoint_idx(self):
        # get the closest waypoints index
        closest_waypoints_idx = self.whole_waypoints_tree.query(self.pose)[1]
        
        # check if the closest waypoint is ahead or behind the ego
        closest_coord = self.whole_waypoints_2d[closest_waypoints_idx]
        prev_coord = self.whole_waypoints_2d[closest_waypoints_idx - 1]
        cl_vect = np.array(closest_coord)
        pre_vect = np.array(prev_coord)
        pos_vect = np.array(self.pose)
        validation = np.dot(cl_vect - pre_vect, cl_vect - pos_vect)
        
        # if the closest waypoint is behind the ego
        if validation < 0:
            closest_waypoints_idx = (closest_waypoints_idx + 1) % len(self.whole_waypoints_2d)
        return closest_waypoints_idx
    
    def publish_waypoints(self, closest_waypoints_idx):
        self.lookahead_waypoints = Lane()
        self.lookahead_waypoints.header = self.whole_waypoints.header
        self.lookahead_waypoints.waypoints = self.whole_waypoints.waypoints[closest_waypoints_idx:closest_waypoints_idx + LOOKAHEAD_WPS]
	print 'etwas..............'
        if self.stopline_waypoints_idx:
	    print "stopline_waypoints_idx: %d", self.stopline_waypoints_idx
        if self.stopline_waypoints_idx != -1 and self.stopline_waypoints_idx < closest_waypoints_idx + LOOKAHEAD_WPS:
	    decelerated_waypoints = self.decelerate_waypoints(self.lookahead_waypoints.waypoints, closest_waypoints_idx)
            decelerated_Lane = Lane()
            decelerated_Lane.header = self.lookahead_waypoints.header
            decelerated_Lane.waypoints = decelerated_waypoints
	    self.final_waypoints_pub.publish(decelerated_Lane)
	else:
            self.final_waypoints_pub.publish(self.lookahead_waypoints)
        
        
    def pose_cb(self, msg):
        # TODO: Implement
        self.pose = [msg.pose.position.x, msg.pose.position.y] 

    def waypoints_cb(self, waypoints):
        # TODO: Implement
        self.whole_waypoints = waypoints
        if not self.whole_waypoints_2d:
            self.whole_waypoints_2d = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] \
                                       for waypoint in waypoints.waypoints]
            self.whole_waypoints_tree = KDTree(self.whole_waypoints_2d)

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        self.stopline_waypoints_idx = msg.data

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist
    
    def decelerate_waypoints(self, waypoints, closest_waypoints_idx):
        decelerated_waypoints = []
        for i, wp in enumerate(waypoints):
            decelerated_wp = Waypoint()
            decelerated_wp.pose = wp.pose
            stop_idx = max(self.stopline_waypoints_idx - closest_waypoints_idx - 5, 0) # Five waypoints back from the stopline
            distance_to_stopline = self.distance(waypoints, i, stop_idx)
            velocity = math.sqrt(2 * DECEL_KOMPORT * distance_to_stopline)
            velocity = 0 if velocity < 1. else velocity
            decelerated_wp.twist.twist.linear.x = min(velocity, wp.twist.twist.linear.x)
            decelerated_waypoints.append(decelerated_wp)
        return decelerated_waypoints


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
