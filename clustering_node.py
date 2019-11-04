#! /usr/bin/env python

import rospy
import math
from sensor_msgs.msg import LaserScan
import numpy as np

from geometry_msgs.msg import *

def callback(msg):

    # transform r, theta to x, y
    rho = np.array(msg.ranges)
    phi = np.arange(360)*math.pi/180

    x = -1 * rho * np.sin(phi)
    y = rho * np.cos(phi)

    indexx0 = np.where(x == 0)
    new_xx = np.delete(x, indexx0)
    new_yy = np.delete(y, indexx0)

    indexx1 = np.where(abs(new_xx) >= 1.5)
    indexy1 = np.where(abs(new_yy) >= 1.5)
    indexxy1 = np.append(indexx1, indexy1)
    index = np.unique(indexxy1)
    new_x = np.delete(new_xx, index)
    new_y = np.delete(new_yy, index)

    point = np.c_[new_x, new_y]
    #print point

    cluster_i = []
    for i in range(len(point)-1):
       if abs(new_x[i]-new_x[i+1]) > 0.1 or abs(new_y[i]-new_y[i+1]) > 0.1:
           cluster_i.append(i)
    #print cluster_i

    cluster = []
    for i in range(len(cluster_i)-1):

        if i == 0:
            a = point[:cluster_i[i]+1]
            cluster.append(a)
        if 0 <= i < len(cluster_i)-1:
            b = point[cluster_i[i]+1:cluster_i[i+1]+1]
            cluster.append(b)
        if i == len(cluster_i)-2:
            c = point[cluster_i[i+1]+1:]
            cluster.append(c)
    #print cluster

    leg_i = []
    x_mids = []
    y_mids = []
    for i in range(len(cluster)):
        x_dis = math.pow((cluster[i][0][0]-cluster[i][-1][0]),2)
        y_dis = math.pow((cluster[i][0][1]-cluster[i][-1][1]),2)
        z = x_dis + y_dis
        width = math.sqrt(z)
        #print "leg width{}, {}".format(width, i)

        if 0.07 < width < 0.15:
            #print "leg width : {}, {}".format(width, i)
            leg_i.append(i)
            #print cluster[i]

            x_mid = (cluster[i][0][0]+cluster[i][-1][0])/2
            x_mids.append(x_mid)
            y_mid = (cluster[i][0][1]+cluster[i][-1][1])/2
            y_mids.append(y_mid)
    #print leg_i, x_mids, y_mids
    #print "========================================================"

    leg_i.insert(0, leg_i[-1])
    x_mids.insert(0, x_mids[-1])
    y_mids.insert(0, y_mids[-1])
    #print leg_i


    for i in range(len(leg_i)-1):

        x_dis2 = math.pow(x_mids[i]-x_mids[i+1],2)
        y_dis2 = math.pow(y_mids[i]-y_mids[i+1],2)
        z2 = x_dis2 + y_dis2
        leg_dis = math.sqrt(z2)
        #print "full{}".format(leg_dis)

        if leg_dis < 0.5:

            #print "leg between width : {}, {}".format(leg_dis, i)
            x_mid2 = (x_mids[i]+x_mids[i+1])/2
            y_mid2 = (y_mids[i]+y_mids[i+1])/2


            print "point x : {}, point y : {}".format(x_mid2, y_mid2)

    print "========="

    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    twist = Twist()

    v = 0.2

    if x_mid2 < -0.05:
        twist.angular.z = v
    elif x_mids > 0.05:
        twist.angular.z = -1 * v

    if 0.3 < y_mid2 < 0.4:
        twist.linear.x = 0
    elif y_mid2 < 0.3:
        twist.linear.x = -1 * v
    elif y_mid2 > 0.4:
        twist.linear.x = v

    twist.linear.y = y_mid2
    twist.linear.z = 0.0

    rospy.loginfo("Sending goal location ...")
    pub.publish(twist)


rospy.init_node('scan_values')
sub = rospy.Subscriber('/scan', LaserScan, callback)
rospy.Duration(5)
rospy.spin()



# if__name__ == "__main__":
#     pass
