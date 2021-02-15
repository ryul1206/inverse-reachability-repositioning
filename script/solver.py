#!/usr/bin/env python
import rospy
import rospkg
import numpy as np
import irm
from collision import CollisionBox
from geometry_msgs.msg import Pose2D
from ir_repositioning.srv import Repositioning, RepositioningResponse


class InverseReachabilitySolver:
    def __init__(self, base_radius, config):
        self.base_radius = base_radius
        self.config = config

        # Load IRM data
        pkg_dir = rospkg.RosPack().get_path("ir_repositioning")
        file_path = "%s/config/%s.npy" % (pkg_dir, self.config)
        self.reposition = irm.InverseReachabilityMap(file_path, self.base_radius)

        rospy.loginfo("IRM Loading:\n%s", self.reposition)

        # Start a service
        rospy.Service(
            "/ir_repositioning/find_positions", Repositioning, self.callback_process
        )

    def callback_process(self, req):
        # INPUT
        Pt = (req.Pt.x, req.Pt.y)
        Obs = [
            CollisionBox(
                (box.center.x, box.center.y), box.center.theta, box.size_x, box.size_y
            )
            for box in req.Obs
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
        self.reposition.calc(Pt, Obs, Cr, Ct, section_def)

        # OUTPUT
        candidates = self.reposition.get_candidates(num=5)
        rospy.loginfo("num_candidates: %s", len(candidates))

        candis = []
        appros = []
        manips = []
        for Ct, Cr, x, y, m in candidates:
            pose = Pose2D()
            pose.x = x
            pose.y = y
            pose.theta = np.radians(Ct - Cr)
            candis.append(pose)
            appros.append(Cr)
            manips.append(m)
        rospy.logwarn(
            "feasi solver:\n\tCt: %s\n\tCr: %s\n\t=> theta: %s",
            candidates[0, 0],
            candidates[0, 1],
            candis[0].theta,
        )

        resp = RepositioningResponse()
        resp.num_candidates = len(candidates)
        resp.candidates = candis
        resp.approach_angles = appros
        resp.manipulabilities = manips
        return resp


def run_service():
    rospy.init_node("ir_server")
    InverseReachabilitySolver(
        float(rospy.get_param("/ir_server/base_radius")),
        rospy.get_param("/ir_server/config"),
    )
    rospy.spin()


if __name__ == "__main__":
    run_service()
