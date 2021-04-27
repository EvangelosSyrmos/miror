#!/usr/bin/env python
from __future__ import print_function
import rospy
import nav_msgs.srv
import math
from std_msgs.msg import Float32, Header
from geometry_msgs.msg import PoseStamped, Point, Pose, Quaternion
from rospy.exceptions import ROSException


def calculate_cost(poses):
    '''
    Calculates the euclidean distance from each consecutive point
    and returns the cost
    '''

    sum = 0
    for pose in poses:
        sum += math.sqrt(math.pow(((pose.pose.position.x + 1)- (pose.pose.position.x)),2)
                         + math.pow(((pose.pose.position.y + 1)- (pose.pose.position.y)),2))
    return sum

def main(x1, y1, x2, y2):
    rospy.init_node("get_paths")
    try:
        rospy.wait_for_service('/move_base/make_plan', timeout=5.0)
    except ROSException:
        print("Service not responding.")
    try:
        get_plan = rospy.ServiceProxy('/move_base/make_plan', nav_msgs.srv.GetPlan)
    except rospy.service.ServiceException as e:
        print("Service failed {}".format(e))

    start = PoseStamped()
    start.header.frame_id = "map"
    start.header.stamp = rospy.Time.now()
    start.pose.position = Point(x1, y1, 0)

    goal = PoseStamped()
    goal.header.frame_id = "map"
    goal.header.stamp = rospy.Time.now()
    goal.pose.position = Point(x2, y2, 0)

    tolerance = Float32()
    tolerance = 0.2

    # Create a Reqeust to the service
    req = nav_msgs.srv.GetPlanRequest(start, goal, tolerance)
    # Get the response with all the points
    resp = get_plan(req)
    print("Poits {}".format(len(resp.plan.poses)))
    # Calculate path's total cost 
    if resp:
        cost = calculate_cost(resp.plan.poses)
    print("The cost is {}".format(cost))

if __name__ == '__main__':
    x1 = -2.02
    y1 = -0.38
    x2 = -1.99
    y2 = -0.45

    main(x1, y1, x2, y2)