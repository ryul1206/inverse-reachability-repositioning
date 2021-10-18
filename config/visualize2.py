import numpy as np
import matplotlib.pyplot as plt
import matplotlib.ticker as plticker
import mpl_toolkits.mplot3d as mp3d
from mpl_toolkits.mplot3d import axes3d
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from tf import transformations


def hex(r, g, b):
    """input: 0~255"""
    return '#%02x%02x%02x' % (r, g, b)


def new_base(Rmat, P, length=0.1):
    base_length = length
    vert = [
        np.array((base_length, 0., 0.)),
        np.array((0., base_length / 3., 0.)),
        np.array((0., -base_length / 3., 0.)),
    ]
    vert = [np.dot(Rmat, v) + P for v in vert]
    return vert


def new_tcp(Rmat, P, length=0.04):
    tcp_length = length
    vert = [
        np.array((0., 0., 0.)),
        np.array((0., 0., tcp_length)),
        np.array((0., -tcp_length * 0.2, tcp_length * 0.7)),
    ]
    vert = [np.dot(Rmat, v) + P for v in vert]
    return vert


def get_manip_normalized(row_data, manip_index):
    manips = [row[manip_index] for row in row_data]
    max_manip = np.max(manips)
    min_manip = np.min(manips)
    d_manip = max_manip - min_manip

    def manip_normalized(m):
        return (m - min_manip) / d_manip

    return manip_normalized


def gradient_color(v):
    """ high is blue """
    # red = ((-2.0 * v + 1.0) if v < 0.5 else 0.0) * 255
    # green = ((2.0 * v) if v < 0.5 else (-2.0 * v + 2.0)) * 255
    # blue = (0.0 if v < 0.5 else (2.0 * v - 1.0)) * 255
    # return hex(red, green, blue)

    # colormap = plt.cm.get_cmap('rainbow')  # high is red
    # return colormap(1.0 - v)

    # colormap = plt.cm.get_cmap('gist_rainbow')  # high is blue
    # return colormap(v)

    colormap = plt.cm.get_cmap('jet')  # high is red
    return colormap(1.0 - v)


def sub_line(xlimit, ylimit, zlimit, ax):
    margin = 0.03
    verts = [
        [(xlimit[0] - margin, 0, 0), (0, 0, 0)],
        [(0, 0, 0), (xlimit[1] + margin, 0, 0)],
        [(0, ylimit[0] - margin, 0), (0, 0, 0)],
        [(0, 0, 0), (0, ylimit[1] + margin, 0)],
        [(0, 0, zlimit[0] - margin), (0, 0, 0)],
        [(0, 0, 0), (0, 0, zlimit[1] + margin)],
    ]
    # srf = Poly3DCollection(verts, edgecolor=hex(100, 100, 100), linewidth=.5)
    for v in verts:
        srf = Poly3DCollection([v], edgecolor=hex(255, 120, 250), linewidth=1.2)
        ax.add_collection3d(srf)


def canvas(ax, xlimit, ylimit, zlimit):
    # elevation = 11  # deg
    # azimuth = 174  # deg

    # elevation = 43  # deg
    # azimuth = 140  # deg

    elevation = 17  # deg
    azimuth = 168  # deg
    # elevation = 0  # deg
    # azimuth = 90  # deg

    ax.view_init(elevation, azimuth)
    # unit: meter
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')
    ax.set_xlim3d(xlimit[0], xlimit[1])
    ax.set_ylim3d(ylimit[0], ylimit[1])
    ax.set_zlim3d(zlimit[0], zlimit[1])
    # ax.set_aspect('equal')
    hor = 1.0
    ver = 1.0
    ax.set_aspect(ver / hor)

    tick_interval = .5 if (xlimit[1] - xlimit[0]) > 2.0 else .25
    ax.xaxis.set_major_locator(plticker.MultipleLocator(base=tick_interval))
    ax.yaxis.set_major_locator(plticker.MultipleLocator(base=tick_interval))
    ax.zaxis.set_major_locator(plticker.MultipleLocator(base=tick_interval))

    ax.tick_params(axis='x', rotation=25)
    ax.tick_params(axis='y', rotation=-60)
    ax.tick_params(axis='z', rotation=0)
    x_ticks = ax.get_xticks()[1:-1]
    y_ticks = ax.get_yticks()[1:-1]
    z_ticks = ax.get_zticks()[1:-1]
    ax.set_xticklabels(x_ticks, verticalalignment='center', horizontalalignment='center')
    ax.set_yticklabels(y_ticks, verticalalignment='center', horizontalalignment='center')
    ax.set_zticklabels(z_ticks, verticalalignment='bottom', horizontalalignment='center')
    return ax


MANY_base_size = 0.07
MANY_base_width = 0.5
ONE_base_size = 0.2
ONE_base_width = 2.5

MANY_tcp_size = 0.1
MANY_tcp_width = .5
ONE_tcp_size = 0.2
ONE_tcp_width = 2.5


def RM(ax, npy_name, xyz_view_boundary):
    xlimit, ylimit, zlimit = xyz_view_boundary
    # xlimit = (-.5, .7)
    # ylimit = (-.9, .3)
    # zlimit = (-.2, 1.)
    ax = canvas(ax, xlimit, ylimit, zlimit)
    rm = np.load(npy_name)

    # COLOR
    IDX_M = 9
    manip_normalized = get_manip_normalized(rm, IDX_M)

    # BASE
    base_verts = [new_base(np.identity(3), np.array((0, 0, 0.02)), length=ONE_base_size)]
    srf = Poly3DCollection(base_verts, alpha=1.0, edgecolor=hex(0, 0, 0), linewidth=ONE_base_width)
    srf.set_facecolor(hex(255, 255, 255))
    ax.add_collection3d(srf)

    # TCP
    tcp_verts = []
    colors = []

    for row in rm:
        P = np.array(row[:3])  # xyz
        yaw = np.radians(row[8])
        pitch = np.radians(row[7])
        roll = np.radians(row[6])
        R = transformations.euler_matrix(yaw, pitch, roll, axes='rzyz')[:3, :3]
        tcp = new_tcp(R, P, length=MANY_tcp_size)
        _m = row[IDX_M]
        mn = manip_normalized(_m)

        tcp_verts.append(tcp)
        colors.append(gradient_color(mn))

    srf = Poly3DCollection(tcp_verts, closed=False, alpha=.0, edgecolor=colors, linewidth=MANY_tcp_width)
    srf.set_facecolor(hex(255, 255, 255))
    ax.add_collection3d(srf)

    # line
    sub_line(xlimit, ylimit, zlimit, ax)


def OLD_IRM(ax, npy_name, xyz_view_boundary):
    xlimit, ylimit, zlimit = xyz_view_boundary
    ax = canvas(ax, xlimit, ylimit, zlimit)
    irm = np.load(npy_name)

    # COLOR
    IDX_M = 10
    manip_normalized = get_manip_normalized(irm, IDX_M)

    # BASE
    vcs = ([], [])  # (verts, colors)
    for row in irm:
        P = np.array(row[:3])  # xyz
        quat = np.array(row[6:10])
        R = transformations.quaternion_matrix(quat)[:3, :3]
        base = new_base(R, P, length=MANY_base_size)
        _m = row[IDX_M]
        mn = manip_normalized(_m)

        vcs[0].append(base)
        vcs[1].append(gradient_color(mn))

    # for vc in vcs:
    base_verts, colors = vcs
    srf = Poly3DCollection(base_verts, alpha=0.0, edgecolor=colors, linewidth=MANY_base_width)
    srf.set_facecolor(colors)
    ax.add_collection3d(srf)

    # TCP
    tcp_verts = [new_tcp(np.identity(3), np.array((-0.02, 0.02, 0.02)), length=ONE_tcp_size)]
    srf = Poly3DCollection(tcp_verts, closed=False, alpha=0.0, edgecolor=hex(0, 0, 0), linewidth=ONE_tcp_width)
    srf.set_facecolor(hex(255, 255, 255))
    ax.add_collection3d(srf)

    # line
    sub_line(xlimit, ylimit, zlimit, ax)


def NEW_IRM(ax, npy_name, xyz_view_boundary):
    xlimit, ylimit, zlimit = xyz_view_boundary
    # xlimit = (-.8, .4)
    # ylimit = (-.5, .7)
    # zlimit = (-.2, 1.)
    ax = canvas(ax, xlimit, ylimit, zlimit)
    irm = np.load(npy_name)

    # COLOR
    IDX_M = 9
    manip_normalized = get_manip_normalized(irm, IDX_M)

    # BASE
    vcs = ([], [])  # (verts, colors)
    for row in irm:
        tcp_yaw = np.radians(row[8])  # Ci
        R = transformations.rotation_matrix(-tcp_yaw, (0, 0, 1))[:3, :3]
        P = np.array((row[0], row[1], 0))  # (x, y, 0)
        base = new_base(R, P, length=MANY_base_size)
        _m = row[IDX_M]
        mn = manip_normalized(_m)

        vcs[0].append(base)
        vcs[1].append(gradient_color(mn))

    # for vc in vcs:
    base_verts, colors = vcs
    srf = Poly3DCollection(base_verts, alpha=0.0, edgecolor=colors, linewidth=MANY_base_width)
    srf.set_facecolor(colors)
    ax.add_collection3d(srf)

    # TCP
    tcp_verts = []
    for row in irm:
        tcp_pitch = np.radians(row[7])
        tcp_roll = np.radians(row[6])
        R = transformations.euler_matrix(0, tcp_pitch, tcp_roll, axes='rzyz')[:3, :3]
        P = np.array((0, 0, row[2]))  # (0, 0, z)
        tcp_verts.append(new_tcp(R, P, length=MANY_tcp_size))
    srf = Poly3DCollection(tcp_verts, closed=False, alpha=0.0, edgecolor=hex(0, 0, 0), linewidth=MANY_tcp_width)
    srf.set_facecolor(hex(255, 255, 255))
    ax.add_collection3d(srf)

    # line
    sub_line(xlimit, ylimit, zlimit, ax)


if __name__ == "__main__":
    fig = plt.figure(figsize=plt.figaspect(1.0 / 3.0))
    # ax = fig.gca(projection='3d')
    ax1 = fig.add_subplot(1, 3, 1, projection='3d', title="RM")
    ax2 = fig.add_subplot(1, 3, 2, projection='3d', title="Reference IRM")
    ax3 = fig.add_subplot(1, 3, 3, projection='3d', title="Proposed IRM")

    xyz_boundary1 = ((-.4, 1.), (-1., .4), (.0, 1.4))       # 1.4, 1.4, 1.4
    xyz_boundary2 = ((-1., 1.4), (-1., 1.4), (-1.3, 1.1))   # 2.4, 2.4, 2.4
    xyz_boundary3 = ((-.7, .7), (-.7, .7), (.0, 1.4))       # 1.4, 1.4, 1.4

    RM(ax1, 'socialrobot_wip_right_rm.npy', xyz_boundary1)
    OLD_IRM(ax2, 'socialrobot_wip_right_old_irm.npy', xyz_boundary2)
    NEW_IRM(ax3, 'socialrobot_wip_right_irm.npy', xyz_boundary3)

    plt.show()


    # =========================================
    # # XY Plane (Z Plane)
    # verts = [[
    #     (xlimit[0], ylimit[0], 0),
    #     (xlimit[0], ylimit[1], 0),
    #     (xlimit[1], ylimit[1], 0),
    #     (xlimit[1], ylimit[0], 0),
    # ]]
    # srf = Poly3DCollection(verts, alpha=.8, facecolor=hex(255, 255, 0))
    # ax.add_collection3d(srf)
    # # YZ Plane (X Plane)
    # verts = [[
    #     (0, ylimit[0], zlimit[0]),
    #     (0, ylimit[1], zlimit[0]),
    #     (0, ylimit[1], zlimit[1]),
    #     (0, ylimit[0], zlimit[1]),
    # ]]
    # srf = Poly3DCollection(verts, alpha=.8, facecolor=hex(0, 255, 255))
    # ax.add_collection3d(srf)
