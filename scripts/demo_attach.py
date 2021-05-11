#!/usr/bin/env python

import rospy
from gazebo_ros_link_attacher.srv import Attach, AttachRequest, AttachResponse


if __name__ == '__main__':
    rospy.init_node('demo_attach_links')
    rospy.loginfo("Creating ServiceProxy to /link_attacher_node/attach")
    attach_srv = rospy.ServiceProxy('/link_attacher_node/attach',
                                    Attach)
    attach_srv.wait_for_service()
    rospy.loginfo("Created ServiceProxy to /link_attacher_node/attach")

    # Link them
    rospy.loginfo("Attaching gripper and red box.")
    req = AttachRequest()
    req.model_name_1 = "robot"
    req.link_name_1 = "wrist_3_link"
    req.model_name_2 = "king_white"
    req.link_name_2 = "link_0"

    attach_srv.call(req)