import numpy as np
import matplotlib.pyplot as plt
from basic_math import rotation_3d_about_z
import feasibility_utils as futils
"""
raw:
    [[Cr x y manip],
     [Cr x y manip], ...]
layers:
    [deg]: [[x, ...],
            [y, ...],
            [manip, ...]]
"""

# For Visualization ##############
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


class FeasibilityMap:
    def __init__(self, manipulability_raw, robot_radius, is_jupyter=False):
        self.robot_radius = robot_radius
        manip_layers = futils.raw_to_layers(manipulability_raw)
        feasi_layers = self.manipulability_to_feasibility(manip_layers)
        self.feasi_raw = futils.layers_to_raw(feasi_layers)
        self.free_raw = None

        self.target_center = np.array([0, 0])
        self._is_jupyter = is_jupyter
        if self._is_jupyter:
            self.xyMinMax = [-0.5, 0.5, -0.5, 0.5]
            futils.two_plot([(manip_layers, False), (self.feasi_raw, True)], self.xyMinMax)
            futils.two_plot([(manip_layers, False), (self.feasi_raw, True)], self.xyMinMax, dim="3d")
            print(manip_layers[0].shape)
            print(feasi_layers[0].shape)
            print(self.feasi_raw.shape)
        else:
            # ROS
            self.rviz = rospy.Publisher("/feasibility/debug_markers", Marker, queue_size=1, latch=True)

    def xy_correction(self, xys):
        """For ROS
        xys: np.array [[x, y], ...]
        """
        corrected = xys + self.target_center
        return corrected

    @staticmethod
    def manipulability_to_feasibility(manipulability_layers):
        """
        manipulability_layers & feasibility_layers
            key: degree
            value: [(x, y, manip), ...]

        [rot]      | [Cr0]
        c0 -s0  0  | x1 x2 x3 x4 x5 ... x100
        s0  c0  0  | y1 y2 y3 y4 y5 ... y100
         0   0  1  | u1 u2 u3 u4 u5 ... u100
        """
        feasibility_layers = {}
        for Cr, mlayer in manipulability_layers.items():
            R = rotation_3d_about_z(np.radians(180 - Cr))
            flayer = np.dot(R, mlayer)
            feasibility_layers[Cr] = flayer
        return feasibility_layers

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
        minCr = np.degrees(Cr[0])
        maxCr = np.degrees(Cr[1])
        if self._is_jupyter:
            print("Cut the range of `Cr` from `Fraw` and set it to `Fcut`.")
            print("min Cr: ", minCr)
            print("max Cr: ", maxCr)
        filter_arr = (self.feasi_raw[:, 0] >= minCr) * (self.feasi_raw[:, 0] <= maxCr)
        Fcut = self.feasi_raw[filter_arr].copy()

        # PLOT
        if self._is_jupyter:
            futils.two_plot([(self.feasi_raw, True), (Fcut, True)], self.xyMinMax)

        ###################################
        if self._is_jupyter:
            print("And extract only the maximum as a `Fmax`.")
        # The unit of section_def is meter.
        min_radius = int(section_def[0] * 100)
        max_radius = int(section_def[1] * 100)
        interval = int(section_def[2] * 100)
        # cm
        sections = np.array(range(min_radius, max_radius, interval))
        # meter
        sections = sections / 100.0
        sq_sections = np.square(sections)
        if self._is_jupyter:
            print("sq_sections.shape: ", sq_sections.shape)

        num_sections = len(sq_sections) - 1
        Fmax = np.zeros((num_sections, 4), dtype=np.float32)

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
            x, y = point[1:3]
            sq_radius = x * x + y * y
            idx = find_idx(sq_sections, sq_radius)
            if idx is not None:
                prev_manip = Fmax[idx, 3]
                new_manip = point[3]
                if prev_manip < new_manip:
                    Fmax[idx] = point.copy()

        # Trim the zero-sections
        trimmed = []
        for point in Fmax:
            manip = point[3]
            if manip > 0:
                trimmed.append(point)
        Fmax = np.array(trimmed)

        if self._is_jupyter:
            print("Fmax.shape: ", Fmax.shape)
            print("Fmax:\n", Fmax)

        # PLOT
        if self._is_jupyter:
            print("When Ct == 0.0 deg")
            fig, (ax1, ax2) = plt.subplots(nrows=1, ncols=2)
            futils.scatter_2d(ax1, Fcut, self.xyMinMax)
            futils.scatter_2d(ax2, Fmax, self.xyMinMax)
            for r in sections:
                ax1.add_artist(plt.Circle((0, 0), radius=r, color="gray", alpha=0.4, fill=False))
                ax2.add_artist(plt.Circle((0, 0), radius=r, color="gray", alpha=0.4, fill=False))
            fig.tight_layout()
            plt.show()
        else:
            # ROS
            _xys = self.xy_correction(Fcut[:, 1:3])
            _points = [(x, y, 0.0) for x, y in _xys]
            _ms = Fcut[:, 3]
            max_manip = np.max(_ms)
            _colors = [futils.manip_color(futils.normalized_value(m, max_manip, 0, 1, 0)) for m in _ms]
            self.rviz.publish(rviz_utils.create_points(
                _id["Fcut"],
                _points,
                _colors,
            ))

        ###################################
        minCt = np.degrees(Ct[0])
        maxCt = np.degrees(Ct[1])
        if self._is_jupyter:
            print("Wipe the `Fmax` in the range of `Ct`. => `Fwiped`")
            print("min Ct: ", minCt)
            print("max Ct: ", maxCt)
        """
        Fwiped
        [Ct Cr x y m]
        """
        interval = 0.02  # m
        two_pi = 2.0 * np.pi

        circle_points = []
        for point in Fmax:
            Cr, x, y, m = point

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
                circle_points.append([Ct, Cr, nx, ny, m])
        Fwiped = np.array(circle_points)
        if self._is_jupyter:
            print("Fwiped.shape: ", Fwiped.shape)
            print("Interval is ", interval, " [m].")
            # print("Fwiped: ", Fwiped)
            # PLOT
            wiped_plot = Fwiped[:, 1:]
            fig, (ax1, ax2) = plt.subplots(nrows=1, ncols=2)
            futils.scatter_2d(ax1, Fmax, self.xyMinMax)
            futils.scatter_2d(ax2, wiped_plot, self.xyMinMax)
            max_manip = np.max(Fmax[:, 3])
            min_manip = np.min(Fmax[:, 3])
            for point in Fmax:
                cr, x, y, m = point
                ax1.add_artist(
                    plt.Circle(
                        (0, 0),
                        radius=np.sqrt(x * x + y * y),
                        color=futils.manip_color(futils.normalized_value(m, max_manip, min_manip, 1, 0)),
                        fill=False,
                    ))
                ax1.add_artist(plt.Circle((x, y), radius=0.2 / 100.0, color="red", fill=False))
                # ax2.add_artist(
                #     plt.Circle((x, y), radius=0.2 / 100.0, color="red", fill=False)
                # )
            fig.tight_layout()
            plt.show()
        else:
            # ROS
            _xys = self.xy_correction(Fwiped[:, 2:4])
            _points = [(x, y, -0.01) for x, y in _xys]
            _colors = [rviz_utils.t_BLUE for _ in _xys]
            self.rviz.publish(rviz_utils.create_points(
                _id["Fwiped"],
                _points,
                _colors,
                size=0.005,
            ))

        ###################################
        if self._is_jupyter:
            print("Remove all obstacle areas from `Fwiped` with the offset of `Rsize`. => `Fclean`")
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
            filter_arr = [not collision.check(p[2:4] + self.target_center) for p in Fclean]
            Fclean = Fclean[filter_arr]
            if self._is_jupyter:
                print("Fclean.shape: ", Fclean.shape)

        self.free_raw = Fclean.copy()

        if self._is_jupyter:
            # PLOT
            clean_plot = Fclean[:, 1:]
            fig, (ax1, ax2) = plt.subplots(nrows=1, ncols=2)
            futils.scatter_2d(ax1, wiped_plot, self.xyMinMax)
            futils.scatter_2d(ax2, clean_plot, self.xyMinMax)
            for p in Fmax:
                cr, x, y, m = p
                ax1.add_artist(plt.Circle((x, y), radius=0.2 / 100.0, color="red", fill=False))
                ax2.add_artist(plt.Circle((x, y), radius=0.2 / 100.0, color="red", fill=False))
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
                self.rviz.publish(_get_line_strip(_id["obs_real"] + idx, _vxys, rviz_utils.WHITE))
                self.rviz.publish(_get_line_strip(_id["obs_offset"] + idx, _oxys, rviz_utils.RED))
                idx += 1
            # cleaned
            # _xys = self.xy_correction(Fclean[:, 1:3])
            # _points = [(x, y, 0.0) for x, y in _xys]
            # _ms = Fclean[:, 3]
            # max_manip = np.max(_ms)

            # ROS
            _xys = self.xy_correction(Fclean[:, 2:4])
            _points = [(x, y, 0.0) for x, y in _xys]
            _ms = Fclean[:, 4]
            max_manip = np.max(_ms)
            _colors = [futils.manip_color(futils.normalized_value(m, max_manip, 0, 1, 0)) for m in _ms]
            self.rviz.publish(rviz_utils.create_points(
                _id["Fclean"],
                _points,
                _colors,
            ))

    def get_candidates(self, num=1):
        """
        self.free_raw (descending order) relative to target coordinates
        [Ct Cr x y m]
        """
        # Sorting in descending order
        ms = -np.transpose(self.free_raw)[4]
        s = ms.argsort()
        Fsort = self.free_raw[s]
        candidates = Fsort[:num].copy()

        if self._is_jupyter:
            print("num: ", num)
            print("Fclean[:%d]:\n" % num, self.free_raw[:num])
            print("Fclean[:%d] (sorted):\n" % num, Fsort[:num])
            # PLOT
            clean_plot = self.free_raw[:, 1:]
            fig, (ax1, ax2) = plt.subplots(nrows=1, ncols=2)
            futils.scatter_2d(ax1, clean_plot, self.xyMinMax)
            for p in candidates:
                ct, cr, x, y, m = p
                ax1.add_artist(plt.Circle((x, y), radius=0.4 / 100.0, color="blue", fill=False))
            fig.tight_layout()
            plt.show()

        # candidates
        _xys = self.xy_correction(candidates[:, 2:4])
        candidates[:, 2:4] = _xys

        if not self._is_jupyter:
            # ROS
            _points = [(xy[0], xy[1], 0.0) for xy in _xys]
            _colors = [rviz_utils.t_PURPLE for _ in _xys]
            self.rviz.publish(rviz_utils.create_points(
                _id["candidates"],
                _points,
                _colors,
                size=0.015,
            ))
            # best_point
            Ct, Cr = candidates[0, 0:2]
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
