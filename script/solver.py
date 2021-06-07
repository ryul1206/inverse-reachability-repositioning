#!/usr/bin/env python
import rospy
import rospkg
import numpy as np
import irm
import copy
from collision import CollisionBox
from geometry_msgs.msg import Pose2D
from sensor_msgs.msg import JointState
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
        rospy.Service("/ir_server/find_positions", Repositioning, self.callback_process)

    def callback_process(self, req):
        # INPUT
        Pt = (req.Pt.x, req.Pt.y)
        Obs = [CollisionBox((box.center.x, box.center.y), box.center.theta, box.size_x, box.size_y) for box in req.Obs]
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
        candidates = self.reposition.get_candidates(num=-1)
        rospy.loginfo("num_candidates: %s", len(candidates))
        """
        Candidates:
        | Column Index | Name                     | Unit       | Remark           |
        | ------------ | ------------------------ | ---------- | ---------------- |
        | 0            | Ct                       | **DEGREE** |                  |
        | 1            | Cr                       | **DEGREE** | EE Yaw           |
        | 2            | Mobile Base x            | meter      | For query output |
        | 3            | Mobile Base y            | meter      | For query output |
        | 4            | Manipulability           | -          |                  |
        | 5            | Target Object z (height) | meter      | For query input  |
        | 6            | EEP x                    | meter      | Based on base_footprint |
        | 7            | EEP y                    | meter      | Based on base_footprint |
        | 8            | EEP z                    | meter      | Based on base_footprint |
        | 9            | EE Roll                  | **DEGREE** | Based on base_footprint |
        | 10           | EE Pitch                 | **DEGREE** | Based on base_footprint |
        | 11           | EE Yaw                   | **DEGREE** | Cr, Based on base_footprint |
        | 12           | Joint_0 value            | radian     |                  |
        | 13           | Joint_1 value            | radian     |                  |
        | >=14         | Joint_2... values        | radian     |                  |
        """

        candis = []  # geometry_msgs/Pose2D[]
        appros = []  # float64[]
        manips = []  # float64[]
        joint_angles = []  # sensor_msgs/JointState[]
        for data in candidates:
            Ct, Cr, x, y, m = data[:5]
            joints = data[12:]

            pose = Pose2D()
            pose.x = x
            pose.y = y
            pose.theta = np.radians(Ct - Cr)
            candis.append(pose)
            appros.append(Cr)
            manips.append(m)
            joint_angles.append(copy.copy(joints))

        print(type(candidates))
        if len(candidates):
            rospy.logwarn(
                "IR_Solver Result Sample:\n\tCt: %s\n\tCr: %s\n\t=> theta: %s",
                candidates[0, 0],
                candidates[0, 1],
                candis[0].theta,
            )

        resp = RepositioningResponse()
        resp.num_candidates = len(candidates)
        resp.candidates = candis
        resp.approach_angles = appros
        resp.manipulabilities = manips
        for js in joint_angles:
            jmsg = JointState()
            jmsg.position = js
            resp.joint_angles.append(jmsg)
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
