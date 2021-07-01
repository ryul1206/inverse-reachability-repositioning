#!/usr/bin/env python
import rospy
import rospkg
import numpy as np
import irm
import copy
import rviz_utils
from data_shape import IDX, wIDX  # wIDX: Fwiped
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

    def solv_one_hand(self, req):
        # INPUT
        Pt = (req.Pt.x, req.Pt.y)
        Obs = [CollisionBox((box.center.x, box.center.y), box.center.theta, box.size_x, box.size_y) for box in req.Obs]

        # min, max (rad)
        Cr = (req.Cr.x, req.Cr.y)
        Ct = (req.Ct.x, req.Ct.y)
        # min_radius, max_radius, interval
        section_def = (
            req.section_definition.x,
            req.section_definition.y,
            req.section_definition.z,
        )

        if req.hand_type == req.RIGHT_HAND:
            is_right = True
        elif req.hand_type == req.LEFT_HAND:
            is_right = False
        else:
            raise NotImplementedError()
        candidates = self.reposition.calc(Pt, Obs, Cr, Ct, section_def, is_right)
        return candidates

    def solv_dual_hand(self, req, verbose=False):
        data_interval = 0.05  # meter

        # Right hand
        # right_Cr = (89., 91.)
        R_Fcut = self.reposition.get_Fcut(89., 91., verbose)

        obj_center = (req.Pt.x, req.Pt.y)
        self.reposition.target_center = np.array(obj_center)
        # self.reposition.debug_F(R_Fcut, rviz_utils.RED, "R_Fcut", 0.01, 0.04)

        R_Fcut_look_fw = R_Fcut.copy()
        R_Fcut_look_fw[:, IDX["TCP_X"]] = -R_Fcut[:, IDX["TCP_Y"]]
        R_Fcut_look_fw[:, IDX["TCP_Y"]] = R_Fcut[:, IDX["TCP_X"]]

        # self.reposition.debug_F(R_Fcut_look_fw, rviz_utils.t_PURPLE, "R_Fcut_rot", 0.05, 0.04)

        # Left hand
        L_Fcut_look_fw = R_Fcut_look_fw.copy()
        L_Fcut_look_fw[:, IDX["TCP_Y"]] *= -1.0

        # object width
        obj_width = req.dual_hand_width
        quotient = np.around(obj_width / data_interval)
        corrected_width = quotient * data_interval
        center_to_hand = corrected_width / 2.0

        R_Fcut_look_fw[:, IDX["TCP_Y"]] -= center_to_hand
        L_Fcut_look_fw[:, IDX["TCP_Y"]] += center_to_hand

        obj_center = (req.Pt.x, req.Pt.y)
        self.reposition.target_center = np.array(obj_center)
        self.reposition.debug_F(R_Fcut_look_fw, rviz_utils.t_BLUE, "R_Fcut_look_fw", 0.03, 0.03)
        self.reposition.debug_F(L_Fcut_look_fw, rviz_utils.t_RED, "L_Fcut_look_fw", 0.04, 0.02)

        # rospy.logwarn(R_Fcut_look_fw.shape)
        # rospy.loginfo(R_Fcut_look_fw)
        # rospy.logwarn(L_Fcut_look_fw.shape)
        # rospy.loginfo(L_Fcut_look_fw)

        # collection
        def make_int_key(raw_data_point):
            salt = 0.01
            # meter
            x = raw_data_point[IDX["TCP_X"]] + salt
            y = raw_data_point[IDX["TCP_Y"]] + salt
            return (int(x * 20.), int(y * 20.))

        l_dict = {make_int_key(p): p for p in L_Fcut_look_fw}

        # rospy.logwarn(l_dict.keys())

        rospy.logwarn(R_Fcut_look_fw.shape)
        Fdual = []
        for r in R_Fcut_look_fw:
            rospy.loginfo(r.shape)
            xy = make_int_key(r)
            if xy in l_dict:
                # rospy.loginfo(xy) ?? wired... since same xy keys occur
                l_manip = l_dict[xy][IDX["M"]]
                r_manip = r[IDX["M"]]
                dual_manip = min(l_manip, r_manip)
                # rospy.loginfo(dual_manip)

                dual = r.copy()
                dual[IDX["M"]] = dual_manip
                Fdual.append(dual)
        Fdual = np.array(Fdual)
        self.reposition.debug_F(Fdual, rviz_utils.GREEN, "Fdual", 0.055, 0.01)

        rospy.logwarn(Fdual.shape)
        # rospy.loginfo(Fdual)

        # Wiped
        minCt = np.degrees(req.Ct.x)
        maxCt = np.degrees(req.Ct.y)
        Fwiped = self.reposition.get_Fwiped(minCt, maxCt, Fdual, verbose, interval=0.01)
        rospy.logerr(Fwiped.shape)
        self.reposition.debug_Fwiped(Fwiped, rviz_utils.BLUE, "Fwiped", 0.0, 0.01)

        # Clean
        Obs = [CollisionBox((box.center.x, box.center.y), box.center.theta, box.size_x, box.size_y) for box in req.Obs]
        Fclean = self.reposition.get_Fclean(Obs, Fwiped, verbose)

        rospy.logwarn(Fclean.shape)

        candidates = self.reposition.get_candidates(Fclean, num=-1, is_dual=True)
        return candidates

    def callback_process(self, req):
        if req.hand_type == req.DUAL_HAND:
            candidates = self.solv_dual_hand(req)
        else:
            candidates = self.solv_one_hand(req)

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
            pose.theta = np.radians((Ct - Cr) if req.hand_type != req.DUAL_HAND else Ct)
            candis.append(pose)
            appros.append(Cr)
            manips.append(m)
            joint_angles.append(copy.copy(joints))

        # print(type(candidates))
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
