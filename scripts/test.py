import rospy

def main():
    rospy.init_node('nrs_node_idle')
    rospy.loginfo("[IdleNode] Node initialized. Waiting indefinitely...")
    rospy.spin()

if __name__ == '__main__':
    main()
