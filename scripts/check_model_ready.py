#!/usr/bin/env python3
"""Return success when a named model exists in the connected Gazebo server."""

import sys

import rospy
from gazebo_msgs.srv import GetModelProperties


def main():
    if len(sys.argv) != 2 or not sys.argv[1]:
        print("usage: check_model_ready.py MODEL_NAME", file=sys.stderr)
        return 2

    rospy.init_node("mecanum_model_readiness", anonymous=True, disable_signals=True)
    try:
        rospy.wait_for_service("/gazebo/get_model_properties", timeout=2.0)
        response = rospy.ServiceProxy(
            "/gazebo/get_model_properties", GetModelProperties
        )(model_name=sys.argv[1])
    except (rospy.ROSException, rospy.ServiceException) as error:
        print(str(error), file=sys.stderr)
        return 1
    if not response.success:
        print(response.status_message, file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
