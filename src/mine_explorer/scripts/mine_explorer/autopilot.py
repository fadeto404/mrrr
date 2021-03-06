#!/usr/bin/env python

import rospy
import roslaunch
import rospkg


def start_launch_file(path):
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    launch = roslaunch.parent.ROSLaunchParent(uuid, [path])
    launch.start()
    rospy.loginfo("Started!")

    return launch


def main():
    rospy.init_node('autopilot')
    rospy.loginfo('Autopilot waiting...')

    rospack = rospkg.RosPack()
    package_path = rospack.get_path('explore_lite')
    launch_path = package_path + '/launch/explore.launch'

    task = None
    exploring = False

    while not rospy.is_shutdown():
        if (exploring) and (task is None):
            rospy.loginfo('Explorer starting')
            task = start_launch_file(launch_path)
        if (not exploring) and (task is not None):
            rospy.loginfo('Explorer shutting down')
            task.shutdown()
            task = None

        exploring = rospy.get_param('/mine_explorer/exploring')

    rospy.spin()
    rospy.loginfo('Shutting down autopilot node')


if __name__ == '__main__':
    main()
