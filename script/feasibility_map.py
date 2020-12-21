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


class FeasibilityMap:
    def __init__(self, manipulability_raw, robot_radius, is_jupyter=False):
        self.robot_radius = robot_radius
        manip_layers = futils.raw_to_layers(manipulability_raw)
        feasi_layers = self.manipulability_to_feasibility(manip_layers)
        self.feasi_raw = futils.layers_to_raw(feasi_layers)
        self.free_raw = None

        self._is_jupyter = is_jupyter
        if self._is_jupyter:
            self.xyMinMax = [-0.5, 0.5, -0.5, 0.5]
            futils.two_plot([(manip_layers, False), (self.feasi_raw, True)], self.xyMinMax)
            futils.two_plot([(manip_layers, False), (self.feasi_raw, True)], self.xyMinMax, dim="3d")
            print(manip_layers[0].shape)
            print(feasi_layers[0].shape)
            print(self.feasi_raw.shape)

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
        Pt: Position of the target object (relative to the robot's current pose)
            - format: (x, y)
        Obs: Area list of ground obstacles
            - format: [CollisionModel, ...]
        Cr: Constraints on the approach angle (relative to the robot heading)
            - format: (min, max)
            - range: -90 ~ 90
        Ct: Constraints on the approach angle (relative to the target heading)
            - format: (min, max)
            - range: -180 ~ 180
        section_def: The definition of ROI (Region of interest)
            - format: (min_radius, max_radius, interval)

        * Distance unit: meters
        * Angle unit: radians
        """
        self.free_raw = None
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

        ###################################
        if self._is_jupyter:
            print("Remove all obstacle areas from `Fwiped` with the offset of `Rsize`. => `Fclean`")
        """
        Fwiped
        [Ct Cr x y m]
        """
        # Pt, Obs
        target_center = np.array(Pt)
        Fclean = Fwiped.copy()
        if self._is_jupyter:
            print("Fclean.shape: ", Fclean.shape)
        for collision in Obs:
            collision.set_offset(self.robot_radius)
            filter_arr = [not collision.check(p[2:4] + target_center) for p in Fclean]
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

    def get_candidates(self, num=1):
        """
        self.free_raw (descending order)
        [Ct Cr x y m]
        """
        # Sorting in descending order
        ms = -np.transpose(self.free_raw)[4]
        s = ms.argsort()
        Fsort = self.free_raw[s]

        if self._is_jupyter:
            print("num: ", num)
            print("Fclean[:%d]:\n" % num, self.free_raw[:num])
            print("Fclean[:%d] (sorted):\n" % num, Fsort[:num])

        candidates = Fsort[:num].copy()

        if self._is_jupyter:
            # PLOT
            clean_plot = self.free_raw[:, 1:]
            fig, (ax1, ax2) = plt.subplots(nrows=1, ncols=2)
            futils.scatter_2d(ax1, clean_plot, self.xyMinMax)
            for p in candidates:
                ct, cr, x, y, m = p
                ax1.add_artist(plt.Circle((x, y), radius=0.4 / 100.0, color="blue", fill=False))
            fig.tight_layout()
            plt.show()

        return candidates
