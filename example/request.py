#!/usr/bin/env python
import numpy as np
import rospy
from vision_msgs.msg import BoundingBox2D
from feasible_mobile_manipulation.srv import FindFeasibility, FindFeasibilityRequest


def request():
    req = FindFeasibilityRequest()
    req.Pt.x = 0.5
    req.Pt.y = -0.3

    box = BoundingBox2D()
    box.center.x = 0.2
    box.center.y = 0.0
    box.center.theta = np.radians(0.0)
    box.size_x = 0.45
    box.size_y = 1.0
    req.Obs = [box]

    req.Cr.x = np.radians(0.0)
    req.Cr.y = np.radians(40.0)
    req.Ct.x = np.radians(-180)
    req.Ct.y = np.radians(180)

    req.section_definition.x = 0.05
    req.section_definition.y = 0.5
    req.section_definition.z = 0.02

    rospy.wait_for_service('/feasibility/find_feasibility')
    try:
        feasi_proxy = rospy.ServiceProxy('/feasibility/find_feasibility', FindFeasibility)
        resp = feasi_proxy(req)
        return resp
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)


if __name__ == "__main__":
    resp = request()
    print(resp)
