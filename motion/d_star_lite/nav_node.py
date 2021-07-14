import rospy

def navigator():
    rospy.init_node('navigation_node', anonymous=True)

    uav_topic = uav+"/mavros/global_position/local"
    goal_topic = uav+"/mavros/setpoint_position/global"
    rospy.Subscriber(uav_topic, Odometry, route_callback, (ID)) 
    rospy.Subscriber(goal_topic, PoseStamped, goal_callback, (ID))


    # spin() -> keeps python from exiting until this node is stopped
    rospy.spin()