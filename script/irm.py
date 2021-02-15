import numpy as np
import matplotlib.pyplot as plt
from jupyter_utils import IDX
import jupyter_utils as jutils

"""
raw:
    [[Cr x y manip],
     [Cr x y manip], ...]
layers:
    [deg]: [[x, ...],
            [y, ...],
            [manip, ...]]
"""

# For Rviz Visualization #########
try:
    import rospy
    from visualization_msgs.msg import Marker
    from tf.transformations import quaternion_from_euler
    import rviz_utils

    _id = {
        # points
        "Fcut": 1000,
        "Fwiped": 1001,
        "Fclean": 1002,
        "candidates": 1003,
        # box
        "obs_real": 2000,
        "obs_offset": 3000,
        # sphere
        "best": 4000,
    }
except ModuleNotFoundError:
    pass
##################################


class InverseReachabilityMap:
    def __init__(self, file_path, robot_radius, is_jupyter=False):
        self.robot_radius = robot_radius
        self.file_path = file_path
        self._is_jupyter = is_jupyter

        # In this file, EE roll, pitch, yaw(Cr) are degree.
        self.irm_raw = np.load(self.file_path)

        max_x = max(abs(self.irm_raw[:, IDX["TCP_X"]])) * 1.1
        max_y = max(abs(self.irm_raw[:, IDX["TCP_Y"]])) * 1.1
        self.xyMinMax = [-max_x, max_x, -max_y, max_y]

        #######
        # TODO: no layer conversion
        # manip_layers = futils.raw_to_layers(manipulability_raw)
        # feasi_layers = self.manipulability_to_feasibility(manip_layers)
        # self.irm_raw = futils.layers_to_raw(feasi_layers)
        #######
        self._wIDX = {
            "Ct": 0,  # Degree
            "Cr": 1,  # Degree
            "Bx": 2,
            "By": 3,
            "M": 4,
            "Tz": 5,
            "EEPx": 6,
            "J": 12,
        }
        self.free_raw = None

        self.target_center = np.array([0, 0])
        if not self._is_jupyter:  # ROS
            self.rviz = rospy.Publisher(
                "/ir_repositioning/debug_markers", Marker, queue_size=1, latch=True
            )

    def __repr__(self):
        repr = "< Inverse Reachability Solver >\n"
        repr += "Configuration:\n"
        repr += "    Base radius: %f m\n" % self.robot_radius
        repr += "    IRM data: %s\n" % self.file_path
        repr += "       shape: %s\n" % str(self.irm_raw.shape)
        return repr

    def xy_correction(self, xys):
        """For ROS
        xys: np.array [[x, y], ...]
        """
        corrected = xys + self.target_center
        return corrected

    def calc(self, Pt, Obs, Cr, Ct, section_def):
        """
        Pt: Position of the target object (in global coordinates)
            - format: (x, y)
        Obs: Area list of ground obstacles (in global coordinates)
            - format: [CollisionModel, ...]
        Cr: Constraints on the approach angle (relative to the robot heading)
            - format: (min, max)
            - range: -90 ~ 90
        Ct: Constraints on the approach angle (in global coordinates)
            - format: (min, max)
            - range: -180 ~ 180
        section_def: The definition of ROI (Region of interest)
            - format: (min_radius, max_radius, interval)

        * Distance unit: meters
        * Angle unit: radians
        """
        self.free_raw = None
        self.target_center = np.array(Pt)

        ###################################
        # 1. Fcut
        ###################################
        minCr = np.degrees(Cr[0])
        maxCr = np.degrees(Cr[1])
        if self._is_jupyter:
            print("Cut the range of `Cr` from `Fraw` and set it to `Fcut`.")
            print("min Cr: ", minCr)
            print("max Cr: ", maxCr)

        Crs = self.irm_raw[:, IDX["Y"]]
        filter_arr = (Crs >= minCr) * (Crs <= maxCr)
        Fcut = self.irm_raw[filter_arr].copy()

        # PLOT
        if self._is_jupyter:
            fig, (ax1, ax2) = plt.subplots(nrows=1, ncols=2)
            jutils.scatter_2d(ax1, jutils.filtering(self.irm_raw), self.xyMinMax)
            jutils.scatter_2d(ax2, jutils.filtering(Fcut), self.xyMinMax)
            fig.tight_layout()
            plt.show()

        ###################################
        # 2. Fmax
        ###################################
        if self._is_jupyter:
            print("And extract only the maximum as a `Fmax`.")

        # meter to cm
        min_radius = int(section_def[0] * 100)
        max_radius = int(section_def[1] * 100)
        interval = int(section_def[2] * 100)
        sections = np.array(range(min_radius, max_radius, interval))
        # cm to meter
        sections = sections / 100.0
        sq_sections = np.square(sections)
        if self._is_jupyter:
            print("sq_sections.shape: ", sq_sections.shape)

        num_sections = len(sq_sections) - 1
        Fmax = np.zeros((num_sections, self.irm_raw.shape[1]), dtype=np.float32)

        def find_idx(ascending, value):
            # Check range
            if ascending[0] > value or ascending[-1] < value:
                return None
            # Binary search
            Li = 0
            Ri = len(ascending) - 1
            while (Li + 1) < Ri:
                mid = int((Li + Ri) / 2)
                if value < ascending[mid]:
                    Ri = mid
                else:
                    Li = mid
            return Li

        for point in Fcut:
            x, y = point[IDX["TCP_X"] : IDX["TCP_Z"]]
            sq_radius = x * x + y * y
            idx = find_idx(sq_sections, sq_radius)
            if idx is not None:
                prev_manip = Fmax[idx, IDX["M"]]
                new_manip = point[IDX["M"]]
                if prev_manip < new_manip:
                    Fmax[idx] = point.copy()

        # Trim the zero-sections
        trimmed = Fmax[:, IDX["M"]] > 0
        Fmax = Fmax[trimmed]

        # PLOT
        if self._is_jupyter:
            print("Fmax.shape: ", Fmax.shape)
            # print("Fmax:\n", Fmax)
            print("When Ct == 0.0 deg")

            fig, (ax1, ax2) = plt.subplots(nrows=1, ncols=2)
            jutils.scatter_2d(ax1, jutils.filtering(Fcut), self.xyMinMax)
            jutils.scatter_2d(ax2, jutils.filtering(Fmax), self.xyMinMax)
            for r in sections:
                ax1.add_artist(
                    plt.Circle((0, 0), radius=r, color="gray", alpha=0.4, fill=False)
                )
                ax2.add_artist(
                    plt.Circle((0, 0), radius=r, color="gray", alpha=0.4, fill=False)
                )
            fig.tight_layout()
            plt.show()
        else:
            # ROS
            _xys = self.xy_correction(Fcut[:, IDX["TCP_X"] : IDX["TCP_Z"]])
            _points = [(x, y, 0.0) for x, y in _xys]
            _ms = Fcut[:, IDX["M"]]
            _colors = jutils.get_colors(_ms)
            self.rviz.publish(rviz_utils.create_points(_id["Fcut"], _points, _colors,))

        ###################################
        # 3. Fwiped
        ###################################
        if self._is_jupyter:
            minCt = np.degrees(Ct[0])
            maxCt = np.degrees(Ct[1])
            print("Wipe the `Fmax` in the range of `Ct`. => `Fwiped`")
            print("min Ct: ", minCt)
            print("max Ct: ", maxCt)
        """
        Fwiped
        [
            Ct,
            Cr,
            base_x,
            base_y,
            m,
            target z,
            eep x, y, z, r, p, y,
            joint 1, 2, 3...
        ]
        """
        wIDX = self._wIDX

        interval = 0.02  # m
        two_pi = 2.0 * np.pi

        circle_points = []
        for point in Fmax:
            Cr_ = point[IDX["Y"]]
            x = point[IDX["TCP_X"]]
            y = point[IDX["TCP_Y"]]
            m_ = point[IDX["M"]]
            target_z_ = point[IDX["TCP_Z"]]
            eep_ = point[IDX["EEP_X"] : IDX["M"]]
            joints_ = point[IDX["Joint"] :]

            radius = np.sqrt(x * x + y * y)
            d_rad = interval / radius

            num_points = int(two_pi / d_rad)
            half = int(num_points / 2)

            # Mapping to -180 ~ 180
            for idx in range(-half, num_points - half):
                rad = d_rad * idx
                Ct = np.degrees(rad)

                c, s = np.cos(rad), np.sin(rad)
                nx = c * x - s * y
                ny = s * x + c * y

                dummy = np.concatenate(
                    ([Ct, Cr_, nx, ny, m_, target_z_], eep_, joints_), axis=0
                )
                circle_points.append(dummy)
                break
        Fwiped = np.array(circle_points)

        if self._is_jupyter:
            print("Fwiped.shape: ", Fwiped.shape)
            print("Interval is ", interval, " [m].")
            fig, (ax1, ax2) = plt.subplots(nrows=1, ncols=2)
            wiped_plot = Fwiped[:, wIDX["Cr"] : wIDX["Tz"]]
            jutils.scatter_2d(ax1, jutils.filtering(Fmax), self.xyMinMax)
            jutils.scatter_2d(ax2, wiped_plot, self.xyMinMax)

            max_manip = np.max(Fmax[:, IDX["M"]])
            min_manip = np.min(Fmax[:, IDX["M"]])
            grad = jutils.ColorGradient()

            for point in Fmax:
                cr = point[IDX["Y"]]
                x = point[IDX["TCP_X"]]
                y = point[IDX["TCP_Y"]]
                m = point[IDX["M"]]
                ax1.add_artist(
                    plt.Circle(
                        (0, 0),
                        radius=np.sqrt(x * x + y * y),
                        color=grad.get(
                            jutils.scale_remapping(m, max_manip, min_manip, 1, 0)
                        ),
                        fill=False,
                    )
                )
                ax1.add_artist(
                    plt.Circle((x, y), radius=0.2 / 100.0, color="red", fill=False)
                )
            fig.tight_layout()
            plt.show()
        else:
            # ROS
            _xys = self.xy_correction(Fwiped[:, wIDX["Bx"] : wIDX["M"]])
            _points = [(x, y, -0.01) for x, y in _xys]
            _colors = [rviz_utils.t_BLUE for _ in _xys]
            self.rviz.publish(
                rviz_utils.create_points(_id["Fwiped"], _points, _colors, size=0.005,)
            )

        ###################################
        # 4. Fclean
        ###################################
        if self._is_jupyter:
            print(
                "Remove all obstacle areas from `Fwiped` with the offset of `Rsize`. => `Fclean`"
            )
        """
        Fwiped
        [Ct Cr x y m]
        """
        # Pt(self.target_center), Obs
        Fclean = Fwiped.copy()
        if self._is_jupyter:
            print("Fclean.shape: ", Fclean.shape)
        for collision in Obs:
            collision.set_offset(self.robot_radius)
            filter_arr = [
                not collision.check(p[wIDX["Bx"] : wIDX["M"]] + self.target_center)
                for p in Fclean
            ]
            Fclean = Fclean[filter_arr]
            if self._is_jupyter:
                print("Fclean.shape: ", Fclean.shape)

        self.free_raw = Fclean.copy()

        if self._is_jupyter:
            # PLOT
            clean_plot = Fclean[:, wIDX["Cr"] : wIDX["Tz"]]
            fig, (ax1, ax2) = plt.subplots(nrows=1, ncols=2)

            jutils.scatter_2d(ax1, wiped_plot, self.xyMinMax)
            if clean_plot.shape[0]:
                jutils.scatter_2d(ax2, clean_plot, self.xyMinMax)
            for p in Fmax:
                cr = p[IDX["Y"]]
                x = p[IDX["TCP_X"]]
                y = p[IDX["TCP_Y"]]
                m = p[IDX["M"]]
                ax1.add_artist(
                    plt.Circle((x, y), radius=0.2 / 100.0, color="red", fill=False)
                )
                if clean_plot.shape[0]:
                    ax2.add_artist(
                        plt.Circle((x, y), radius=0.2 / 100.0, color="red", fill=False)
                    )
            for collision in Obs:
                xs, ys = collision.vertices
                xs = np.append(xs, xs[0])
                ys = np.append(ys, ys[0])
                ax1.plot(xs, ys)
                ax2.plot(xs, ys)
                xs, ys = collision.offsets
                xs = np.append(xs, xs[0])
                ys = np.append(ys, ys[0])
                ax1.plot(xs, ys)
                ax2.plot(xs, ys)
            fig.tight_layout()
            plt.show()
        else:
            # ROS
            def _get_line_strip(_id, vertices, color):
                """vertices: [[x, y], ...]"""
                closed = np.append(vertices, [vertices[0]], axis=0)
                _points = [(x, y, 0.0) for x, y in closed]
                _colors = [color for _ in closed]
                return rviz_utils.create_line(_id, _points, _colors)

            # obstacles
            idx = 0
            for collision in Obs:
                _vxys = np.transpose(collision.vertices)
                _oxys = np.transpose(collision.offsets)
                self.rviz.publish(
                    _get_line_strip(_id["obs_real"] + idx, _vxys, rviz_utils.WHITE)
                )
                self.rviz.publish(
                    _get_line_strip(_id["obs_offset"] + idx, _oxys, rviz_utils.RED)
                )
                idx += 1

            # ROS
            _xys = self.xy_correction(Fclean[:, wIDX["Bx"] : wIDX["M"]])
            _points = [(x, y, 0.0) for x, y in _xys]
            _ms = Fclean[:, wIDX["M"]]
            _colors = jutils.get_colors(_ms)
            self.rviz.publish(
                rviz_utils.create_points(_id["Fclean"], _points, _colors,)
            )

    def get_candidates(self, num=1):
        """
        self.free_raw (descending order) relative to target coordinates
        [Ct Cr x y m]
        """
        wIDX = self._wIDX
        # self._wIDX = {
        #     "Ct": 0,  # Degree
        #     "Cr": 1,  # Degree
        #     "Bx": 2,
        #     "By": 3,
        #     "M": 4,
        #     "Tz": 5,
        #     "EEPx": 6,
        #     "J": 12,
        # }
        # Sorting in descending order
        ms = -np.transpose(self.free_raw)[wIDX["M"]]
        s = ms.argsort()
        Fsort = self.free_raw[s]
        candidates = Fsort[:num].copy()

        if self._is_jupyter:
            print("num: ", num)
            print("Fclean[:%d]:\n" % num, self.free_raw[:num])
            print("Fclean[:%d] (sorted):\n" % num, Fsort[:num])
            # PLOT
            clean_plot = self.free_raw[:, wIDX["Cr"] : wIDX["Tz"]]
            fig, (ax1, ax2) = plt.subplots(nrows=1, ncols=2)
            jutils.scatter_2d(ax1, clean_plot, self.xyMinMax)
            for p in candidates:
                ct, cr, x, y, m = p
                ax1.add_artist(
                    plt.Circle((x, y), radius=0.4 / 100.0, color="blue", fill=False)
                )
            fig.tight_layout()
            plt.show()

        # candidates
        _xys = self.xy_correction(candidates[:, wIDX["Bx"] : wIDX["M"]])
        candidates[:, wIDX["Bx"] : wIDX["M"]] = _xys

        if not self._is_jupyter:
            # ROS
            _points = [(xy[0], xy[1], 0.0) for xy in _xys]
            _colors = [rviz_utils.t_PURPLE for _ in _xys]
            self.rviz.publish(
                rviz_utils.create_points(
                    _id["candidates"], _points, _colors, size=0.015,
                )
            )
            # best_point
            Ct, Cr = candidates[0, : wIDX["Bx"]]
            theta = np.radians(Ct - Cr)
            length = 0.5
            width = 0.02
            height = 0.02
            best_point = rviz_utils.create_marker(
                _id["best"],
                _points[0],
                quaternion_from_euler(theta, 0, 0, axes="rzxy"),
                (length, width, height),
                rviz_utils.PURPLE,
                Marker.ARROW,
            )
            self.rviz.publish(best_point)

        return candidates
