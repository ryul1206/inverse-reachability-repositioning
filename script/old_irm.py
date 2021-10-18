import numpy as np
import rospy
from data_shape import OLD_IDX
from tf import transformations


def get_T(R, P):
    """R (4, 4), P (3,)"""
    T = R
    T[:3, 3] = P
    return T


class OldIrm:
    def __init__(self, file_path):
        self.file_path = file_path

        self.irm_raw = np.load(self.file_path)

    def __repr__(self):
        msg = "< Inverse Reachability Solver (OLD Style) >\n"
        msg += "Configuration:\n"
        msg += "    IRM data: %s\n" % self.file_path
        msg += "       Shape: %s\n" % str(self.irm_raw.shape)
        return msg

    def get_Fground(self, Tmat):
        # self.irm_raw.copy()
        Fground = []
        for row in self.irm_raw:
            # Transformation
            tcp_P_base = row[OLD_IDX["Mobile_X"]:(OLD_IDX["Mobile_Z"] + 1)]
            tcp_Quat_base = row[OLD_IDX["Mobile_Qx"]:(OLD_IDX["Mobile_Qw"] + 1)]
            tcp_base_R = transformations.quaternion_matrix(tcp_Quat_base)
            tcp_base_T = get_T(tcp_base_R, tcp_P_base)
            # Tmat == qframe_tcp_T
            qframe_base_T = np.dot(Tmat, tcp_base_T)

            tcp_P_eef = row[OLD_IDX["EEP_X"]:(OLD_IDX["EEP_Z"] + 1)]
            tcp_eef_T = get_T(np.identity(4), tcp_P_eef)
            qframe_eef_T = np.dot(Tmat, tcp_eef_T)

            manip_scalar = row[OLD_IDX["M"]]
            print("type of manip scalar")
            print(type(manip_scalar))
            exit()
            joints = row[OLD_IDX["Joint"]:]

            Fground.append([qframe_base_T, qframe_eef_T, manip_scalar, joints])
        Fground = np.array(Fground)

        # Z filter
        Zs = [row[0][2, 3] for row in Fground]
        # up-side and down-side from ground
        closest_upper = np.inf
        closest_lower = -np.inf
        for z in Zs:
            if z >= 0.0:
                closest_upper = z if z < closest_upper else closest_upper
            else:
                closest_lower = z if z > closest_lower else closest_lower
        closest_upper += 0.01
        closest_lower -= 0.01
        z_filter = (Zs >= closest_lower) * (Zs <= closest_upper)
        Fground = Fground[z_filter]

        # Orientation filter (compare Z-vector angle)
        inner_angle_threshold = np.radians(5)
        inner_angles = []
        for row in Fground:
            qframe_base_T = row[0]
            base_P_vecs = np.array([[0, 0, 1, 1], [0, 0, 0, 1]])  # [baseZunit, baseOrigin]
            qframe_P_vecs = np.dot(qframe_base_T, base_P_vecs.transpose())[:3].transpose()
            qframe_P_baseZunit, qframe_P_baseOrigin = qframe_P_vecs
            qframe_P_baseZvec = qframe_P_baseZunit - qframe_P_baseOrigin

            # qframe_P_Zunit = np.array((0, 0, 1))
            # np.dot(qframe_P_baseZvec, qframe_P_Zunit) = qframe_P_baseZvec[2]
            # |qframe_P_baseZvec|*|qframe_P_Zunit| = np.linalg.norm(qframe_P_baseZvec)
            cos_inner = qframe_P_baseZvec[2] / np.linalg.norm(qframe_P_baseZvec)
            _angle = np.arccos(cos_inner)
            inner_angles.append(_angle)
        inner_angles = np.array(inner_angles)
        Fground = Fground[inner_angles < inner_angle_threshold]
        return Fground

    def get_Fcut(self, Fground, minCr_rad, maxCr_rad, max_dist, verbose):
        """
        Assume {tcp} == {eef}
        base_tcp_R
            = base_qframe_R * qframe_tcp_R
            = np.linalg.inv(qframe_base_R) * qframe_tcp_R
            = qframe_base_R.transpose() * qframe_eef_R
        """
        Fcut = []
        for row in Fground:
            qframe_base_T, qframe_eef_T, manip, joints = row
            qframe_base_R = qframe_base_T[:3, :3]
            qframe_eef_R = qframe_eef_T[:3, :3]
            base_tcp_R = np.dot(qframe_base_R.transpose(), qframe_eef_R)

            # Find Cr
            tcp_P_tcpZvec = np.array((0, 0, 1))
            base_P_tcpZvec = np.dot(base_tcp_R, tcp_P_tcpZvec)
            tcpZ_x, tcpZ_y, _ = base_P_tcpZvec
            Cr_rad = np.arctan2(tcpZ_y, tcpZ_x)  # when (0, 0), Cr_rad == 0.0

            # Cr filter
            is_inCr = (Cr_rad >= minCr_rad) and (Cr_rad <= maxCr_rad)
            # max dist
            base_x, base_y = qframe_base_T[:2, 3]
            is_inDist = (base_x**2 + base_y**2) <= (max_dist**2)

            if is_inCr and is_inDist:
                qframe_P_eef = qframe_eef_T[:3, 3]
                qframe_Q_eef = transformations.quaternion_from_matrix(qframe_eef_T)
                data = [
                    0.0,              # Ct_rad, (Ct = 0 in Fcut, Fmax)
                    Cr_rad,           # Cr_rad,
                    base_x,           # qframe_P_baseX,
                    base_y,           # qframe_P_baseY,
                    manip,            # m,
                    qframe_P_eef[0],  # eep x, y, z, qx, qy, qz, qw
                    qframe_P_eef[1],
                    qframe_P_eef[2],
                    qframe_Q_eef[0],
                    qframe_Q_eef[1],
                    qframe_Q_eef[2],
                    qframe_Q_eef[3],
                ]
                data = np.concatenate((data, joints))  # joint 1, 2, 3...
                Fcut.append(data)
        Fcut = np.array(Fcut)
        return Fcut

    def get_Fwiped(self, minCt_rad, maxCt_rad, Fcut, verbose, interval=0.03):
        two_pi = 2.0 * np.pi
        circle_points = []
        for point in Fcut:
            # qframe
            _, Cr_rad, base_x, base_y = point[:4]
            remains = point[4:]

            radius = np.sqrt(base_x**2 + base_y**2)
            d_rad = interval / radius

            num_points = int(two_pi / d_rad)
            half = int(num_points / 2)
            # Circular mapping in the range of Ct
            for idx in range(-half, num_points - half):
                new_Ct_rad = d_rad * idx
                if (new_Ct_rad > minCt_rad) and (new_Ct_rad < maxCt_rad):
                    c, s = np.cos(new_Ct_rad), np.sin(new_Ct_rad)
                    new_base_x = c * base_x - s * base_y
                    new_base_y = s * base_x + c * base_y
                    dummy = np.concatenate(
                        ([new_Ct_rad, Cr_rad, new_base_x, new_base_y], remains), axis=0)
                    circle_points.append(dummy)
        Fwiped = np.array(circle_points)
        return Fwiped

    def get_Fclean(self, Obs, Fwiped, collision_offset, verbose):
        Fclean = Fwiped
        for collision in Obs:
            collision.set_offset(collision_offset)
            filter_arr = [not collision.check(qframe_P_baseXY) for qframe_P_baseXY in Fwiped[:, 2:4]]
            Fclean = Fclean[filter_arr]
        return Fclean

    def get_candidates(self, Fclean, num=-1):
        # Sorting in descending order
        manips = -np.transpose(Fclean)[4]
        s = manips.argsort()
        Fsort = Fclean[s]
        if num >= 0:
            # Collect top N points
            candidates = Fsort[:num].copy()
        else:
            # Collect all points of maximum manipulability.
            idx = 0
            manip_idx = 4
            prev_M = Fsort[0][manip_idx]
            for p in Fsort:
                if prev_M > p[manip_idx]:
                    break
                idx += 1
            candidates = Fsort[:idx].copy()

        if candidates is None:
            candidates = []
        return candidates

    def calc(self, Pt, Obs, Cr, Ct, section_def, collision_offset, max_dist, verbose=False):
        # (x, y, z): target xyz in query_frame({base_footprint} or {global})
        self.target_center = np.array(Pt)
        target_pitch = np.radians(90)
        target_roll = np.radians(-90)

        ##########
        # 0. Validation
        ##########
        qframe_initTcp_Rzyz = transformations.euler_matrix(0, target_pitch, target_roll, axes='rzyz')
        target_xyz = self.target_center
        # T matrix of (initial key pose of TCP -> target pose of TCP)
        Tmat = get_T(qframe_initTcp_Rzyz, target_xyz)  # qframe_tcp_T
        Fground = self.get_Fground(Tmat)

        # index of Fground
        """
        Fground [[
            qframe_base_T (4, 4),
            qframe_eef_T (4, 4),
            m,
            joints
        ], ...]
        """
        ##########
        # 1. Fcut
        ##########
        minCr_rad, maxCr_rad = Cr  # rad
        Fcut = self.get_Fcut(Fground, minCr_rad, maxCr_rad, max_dist, verbose)
        rospy.logwarn("Fcut: %s", Fcut.shape)

        # index of Fcut
        # == index of Fmax (skip)
        # == index of Fwiped
        """
        Fwiped [[
            Ct_rad, (Ct = 0 in Fcut, Fmax)
            Cr_rad,
            qframe_P_baseX,
            qframe_P_baseY,
            m,
            eep x, y, z, qx, qy, qz, qw,
            joint 1, 2, 3...
        ], ...]
        """
        ##########
        # 2. Fmax (skip)
        # 3. Fwiped
        ##########
        minCt_rad, maxCt_rad = Ct
        Fwiped = self.get_Fwiped(minCt_rad, maxCt_rad, Fcut, verbose)
        rospy.logwarn("Fwiped: %s", Fwiped.shape)

        ##########
        # 4. Fclean
        ##########
        Fclean = self.get_Fclean(Obs, Fwiped, collision_offset, verbose)
        rospy.logwarn("Fclean: %s", Fclean.shape)

        # candidates
        candidates = self.get_candidates(Fclean, num=-1)
        return candidates
