#!/usr/bin/env python
import rospy
import rospkg
import numpy as np
import feasibility_map as fmap
from collision import CollisionBox
from geometry_msgs.msg import Pose2D
from feasible_mobile_manipulation.srv import FindFeasibility, FindFeasibilityResponse


class FeasibilitySolver:
    def __init__(self, base_radius, config):
        self.base_radius = base_radius
        self.config = config

        # LOAD DATA
        path = rospkg.RosPack().get_path('feasible_mobile_manipulation')
        M = np.load('%s/config/%s.npy' % (path, self.config))  # In this file, Cr is radians.
        M[:, 0] = np.round(np.degrees(M[:, 0]), decimals=3)  # So, convert it to degrees.
        self._M_shape = M.shape

        # PREPARATION
        self.F = fmap.FeasibilityMap(M, self.base_radius)

        # START SERVICE
        rospy.Service('~find_feasibility', FindFeasibility, self.callback_process)

    def __repr__(self):
        repr = "FeasibilitySolver\n"
        repr += "Configuration:\n"
        repr += "    base_radius: %f m\n" % self.base_radius
        repr += "    config: %s (.npy)\n" % self.config
        repr += "        shape: %s\n" % str(self._M_shape)
        repr += "Feasibility:\n"
        repr += "    feasibility raw.shape: %s\n" % str(self.F.feasi_raw.shape)
        return repr

    def callback_process(self, req):
        # INPUT
        Pt = (req.Pt.x, req.Pt.y)
        Obs = [
            CollisionBox(
                (box.center.x, box.center.y),
                box.center.theta,
                box.size_x,
                box.size_y,
            ) for box in req.Obs
        ]
        # min, max
        Cr = (req.Cr.x, req.Cr.y)
        Ct = (req.Ct.x, req.Ct.y)
        # min_radius, max_radius, interval
        section_def = (
            req.section_definition.x,
            req.section_definition.y,
            req.section_definition.z,
        )

        # PROCESS
        self.F.calc(Pt, Obs, Cr, Ct, section_def)

        # OUTPUT
        candidates = self.F.get_candidates(num=5)
        rospy.loginfo(candidates)

        candis = []
        appros = []
        manips = []
        for Ct, Cr, x, y, m in candidates:
            pose = Pose2D()
            pose.x = x
            pose.y = y
            pose.theta = Ct - Cr
            candis.append(pose)
            appros.append(Cr)
            manips.append(m)

        resp = FindFeasibilityResponse()
        resp.num_candidates = len(candidates)
        resp.candidates = candis
        resp.approach_angles = appros
        resp.manipulabilities = manips
        return resp


def run_service():
    rospy.init_node('feasibility_solver')
    solver = FeasibilitySolver(
        float(rospy.get_param("~base_radius", "0.3")),
        rospy.get_param("~config", "robocare_right_arm_sample"),
    )
    rospy.loginfo(solver)
    rospy.spin()


if __name__ == "__main__":
    run_service()
