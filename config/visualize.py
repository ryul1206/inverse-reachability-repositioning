from colour import hash_or_str
import numpy as np
import matplotlib.pyplot as plt
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
        [(xlimit[0] - margin, 0, 0), (xlimit[1] + margin, 0, 0)],
        [(0, ylimit[0] - margin, 0), (0, ylimit[1] + margin, 0)],
        [(0, 0, zlimit[0] - margin), (0, 0, zlimit[1] + margin)],
    ]
    # srf = Poly3DCollection(verts, edgecolor=hex(100, 100, 100), linewidth=.5)
    srf = Poly3DCollection(verts, edgecolor=hex(255, 120, 250), linewidth=1)
    ax.add_collection3d(srf)


def canvas(ax, xlimit, ylimit, zlimit):
    elevation = 17  # deg
    azimuth = 169  # deg
    ax.view_init(elevation, azimuth)
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

    xticks = ax.get_xticks()
    ax.set_xticklabels(xticks, rotation=20, verticalalignment='bottom', horizontalalignment='center')
    return ax


def RM(ax):
    xlimit = (-.5, .7)
    ylimit = (-.9, .3)
    zlimit = (-.2, 1.)
    ax = canvas(ax, xlimit, ylimit, zlimit)
    rm = np.load('robocare_right_RM.npy')

    # COLOR
    IDX_M = 9
    manip_normalized = get_manip_normalized(rm, IDX_M)

    # BASE
    base_verts = [new_base(np.identity(3), np.zeros(3), length=0.3)]
    srf = Poly3DCollection(base_verts, alpha=1.0, edgecolor=hex(0, 0, 0), linewidth=2.5)
    srf.set_facecolor(hex(255, 255, 255))
    ax.add_collection3d(srf)

    # TCP
    sections = {}  # [z_idx] = (verts, colors)
    sections[0.7] = ([], [])
    sections[0.75] = ([], [])
    sections[0.8] = ([], [])

    for row in rm:
        P = np.array(row[:3])  # xyz
        yaw = np.radians(row[8])
        pitch = np.radians(row[7])
        roll = np.radians(row[6])
        R = transformations.euler_matrix(yaw, pitch, roll, axes='rzyz')[:3, :3]
        tcp = new_tcp(R, P, length=0.15)
        _m = row[IDX_M]
        mn = manip_normalized(_m)

        sec_z = sections[P[2]]
        sec_z[0].append(tcp)
        sec_z[1].append(gradient_color(mn))

    for sec_z in sections.values():
        tcp_verts, colors = sec_z
        srf = Poly3DCollection(tcp_verts, closed=False, alpha=.0, edgecolor=colors, linewidth=.3)
        srf.set_facecolor(hex(255, 255, 255))
        ax.add_collection3d(srf)

    # line
    sub_line(xlimit, ylimit, zlimit, ax)


def OLD_IRM(ax):
    xlimit = (-.7, .5)
    ylimit = (-.2, 1.)
    zlimit = (-.8, .4)
    ax = canvas(ax, xlimit, ylimit, zlimit)
    irm = np.load('robocare_right_old_irm.npy')

    # COLOR
    IDX_M = 10
    manip_normalized = get_manip_normalized(irm, IDX_M)

    # BASE
    vcs = ([], [])  # (verts, colors)
    for row in irm:
        P = np.array(row[:3])  # xyz
        quat = np.array(row[6:10])
        R = transformations.quaternion_matrix(quat)[:3, :3]
        base = new_base(R, P, length=0.3)
        _m = row[IDX_M]
        mn = manip_normalized(_m)

        vcs[0].append(base)
        vcs[1].append(gradient_color(mn))

    # for vc in vcs:
    base_verts, colors = vcs
    srf = Poly3DCollection(base_verts, alpha=0.0, edgecolor=colors, linewidth=.3)
    srf.set_facecolor(colors)
    ax.add_collection3d(srf)

    # TCP
    tcp_verts = [new_tcp(np.identity(3), np.zeros(3), length=0.15)]
    srf = Poly3DCollection(tcp_verts, closed=False, alpha=0.0, edgecolor=hex(0, 0, 0), linewidth=2.5)
    srf.set_facecolor(hex(255, 255, 255))
    ax.add_collection3d(srf)

    # line
    sub_line(xlimit, ylimit, zlimit, ax)


def NEW_IRM(ax):
    xlimit = (-.8, .4)
    ylimit = (-.5, .7)
    zlimit = (-.2, 1.)
    ax = canvas(ax, xlimit, ylimit, zlimit)
    irm = np.load('robocare_right_irm.npy')

    # COLOR
    IDX_M = 9
    manip_normalized = get_manip_normalized(irm, IDX_M)

    # BASE
    vcs = ([], [])  # (verts, colors)
    for row in irm:
        tcp_yaw = np.radians(row[8])  # Ci
        R = transformations.rotation_matrix(-tcp_yaw, (0, 0, 1))[:3, :3]
        P = np.array((row[0], row[1], 0))  # (x, y, 0)
        base = new_base(R, P, length=0.3)
        _m = row[IDX_M]
        mn = manip_normalized(_m)

        vcs[0].append(base)
        vcs[1].append(gradient_color(mn))

    # for vc in vcs:
    base_verts, colors = vcs
    srf = Poly3DCollection(base_verts, alpha=0.0, edgecolor=colors, linewidth=.3)
    srf.set_facecolor(colors)
    ax.add_collection3d(srf)

    # TCP
    tcp_verts = []
    for row in irm:
        tcp_pitch = np.radians(row[7])
        tcp_roll = np.radians(row[6])
        R = transformations.euler_matrix(0, tcp_pitch, tcp_roll, axes='rzyz')[:3, :3]
        P = np.array((0, 0, row[2]))  # (0, 0, z)
        tcp_verts.append(new_tcp(R, P, length=0.15))
    srf = Poly3DCollection(tcp_verts, closed=False, alpha=0.0, edgecolor=hex(0, 0, 0), linewidth=2.5)
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

    RM(ax1)
    OLD_IRM(ax2)
    NEW_IRM(ax3)

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

    # =========================================
    # a = np.array([( 0, 0, 0),( 1, 0, 0),( 1, 1, 0),( 0, 1, 0)])
    # R1 = np.array([[0,-1,0],[1,0,0],[0,0,1]])
    # R2 = (R1[::-1].T)[:,[1,0,2]]
    # R3 = (R1[::-1])[:,[1,0,2]]
    # f = lambda a,r: np.matmul(r, a.T).T
    # g = lambda a,r: [a, f(a,r), f(f(a,r),r), f(f(f(a,r),r),r)]
    # ax.scatter([-1,1], [-1,1], [-1,1], alpha=0.0)
    # for i, ind , r in zip(range(3),[[0,1,2],[2,0,1],[1,2,0]], [R1,R2,R3]):
    #     xy = g(a[:,ind], r )
    #     for x in xy:
    #         face1 = mp3d.art3d.Poly3DCollection([x] , alpha=0.1, linewidth=1)
    #         face1.set_facecolor((i//2, i%2, i==0,  0.5))
    #         ax.add_collection3d(face1)

    # old_irm = np.load('robocare_right_old_irm.npy')
    # new_irm = np.load('robocare_right_irm.npy')
