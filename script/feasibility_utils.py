import numpy as np
import matplotlib.pyplot as plt

# from mpl_toolkits.mplot3d import Axes3D


"""
raw:
    [[Cr x y manip],
     [Cr x y manip], ...]
layers:
    [deg]: [[x, ...],
            [y, ...],
            [manip, ...]]
"""


def recarray_to_ndarray(recarray):
    new_shape = recarray.shape + (-1,)  # (225, -1) = (225,) + (-1,)
    ndarray = recarray.view(np.float32).reshape(new_shape)
    return ndarray


def raw_to_layers(raw):
    def extract_specific_Cr(raw, Cr):
        filter_arr = raw[:, 0] == Cr
        layer = raw[filter_arr]
        layer = layer[:, 1:].copy()
        return np.transpose(layer)

    layers = {}
    for Cr in set(raw[:, 0]):
        layers[Cr] = extract_specific_Cr(raw, Cr)
    return layers


def layers_to_raw(layers):
    num_points = 0
    for Cr, layer in layers.items():
        num_points += layer.shape[1]
    raw = np.zeros((num_points, 4), dtype=np.float32)
    idx = 0
    for Cr, layer in layers.items():
        next_idx = idx + layer.shape[1]
        layer_raw = raw[idx:next_idx]
        layer_raw[:, 0] += Cr
        layer_raw[:, 1:] = np.transpose(layer)
        idx = next_idx
    return raw


def manip_color(manip):
    return (1.0 - manip, manip, 0.0, 1.0)


def normalized_value(value, max_value, min_value, max_target, min_target):
    gain = (value - min_value) / (max_value - min_value)
    return min_target + gain * (max_target - min_target)


def scatter_2d(ax, data, xyMinMax, is_feasibility=True, dot_size=1):
    """
        [Example]
    fig, (ax1, ax2) = plt.subplots(nrows=1, ncols=2)
    xyminmax = [-20, 20, -20, 20]
    scatter_2d(ax1, manip_layers, xyminmax, is_feasibility=False)
    scatter_2d(ax2, self.feasi_raw, xyminmax)
    fig.tight_layout()
    plt.show()
    """
    if type(data) is dict:  # layers
        for Cr, layer in data.items():
            xs, ys, ms = layer
            max_manip = np.max(ms)
            colors = np.array(
                [manip_color(normalized_value(m, max_manip, 0, 1, 0)) for m in ms]
            )
            ax.scatter(xs, ys, c=colors, s=dot_size)
        dkeys = [int(k) for k in data.keys()]
    elif data.shape[1] == 4:  # raw
        raw = np.transpose(data)
        Crs, xs, ys, ms = raw
        max_manip = np.max(ms)
        colors = np.array(
            [manip_color(normalized_value(m, max_manip, 0, 1, 0)) for m in ms]
        )
        ax.scatter(xs, ys, c=colors, s=dot_size)
        dkeys = [int(k) for k in set(Crs)]
    else:
        raise TypeError()
    ax.set_title(
        "%s $C_{r}(deg)$:\n%s"
        % ("Feasibility" if is_feasibility else "Manipulability", dkeys)
    )
    ax.set_xlabel("x(m)", fontsize=15)
    ax.set_ylabel("y(m)", fontsize=15)
    ax.axis(xyMinMax)
    ax.grid(True)

    # robot and object
    if is_feasibility:  # object
        marker_x = [1.0, -0.5, -0.5, -0.5]
        marker_y = [0.0, 0.0, -0.5, 0.5]
    else:  # robot
        marker_x = [1.0, -0.5, -0.5, 1.0]
        marker_y = [0.0, 0.5, -0.5, 0.0]
    arrow_size = max(xyMinMax) * 0.1
    arrow_x = np.array(marker_x) * arrow_size
    arrow_y = np.array(marker_y) * arrow_size
    ax.plot(arrow_x, arrow_y)


def scatter_3d(ax, data, xyMinMax, is_feasibility=True, dot_size=2):
    """
        [Example]
    fig = plt.figure()
    ax1 = fig.add_subplot(121, projection='3d')
    ax2 = fig.add_subplot(122, projection='3d')
    xyminmax = [-20, 20, -20, 20]
    scatter_3d(ax1, manip_layers, xyminmax, is_feasibility=False)
    scatter_3d(ax2, self.feasi_raw, xyminmax)
    plt.show()
    """
    if type(data) is dict:  # layers
        dkeys = [int(k) for k in data.keys()]
        raw = layers_to_raw(data)
    elif data.shape[1] == 4:  # raw
        dkeys = [int(k) for k in set(data[:, 0])]
        raw = data
    else:
        raise TypeError()
    Crs, xs, ys, ms = np.transpose(raw)
    max_manip = np.max(ms)
    colors = np.array(
        [manip_color(normalized_value(m, max_manip, 0, 1, 0)) for m in ms]
    )
    ax.scatter(xs, ys, zs=Crs, c=colors, s=dot_size)
    ax.set_xlabel("x(m)", fontsize=15)
    ax.set_ylabel("y(m)", fontsize=15)
    ax.set_zlabel("$C_{r}$(deg)", fontsize=15)
    ax.axis(xyMinMax)
    ax.set_title("Feasibility" if is_feasibility else "Manipulability")

    # robot and object
    if is_feasibility:  # object
        marker_x = [1.0, -0.5, -0.5, -0.5]
        marker_y = [0.0, 0.0, -0.5, 0.5]
    else:  # robot
        marker_x = [1.0, -0.5, -0.5, 1.0]
        marker_y = [0.0, 0.5, -0.5, 0.0]
    arrow_size = max(xyMinMax) * 0.1
    arrow_x = np.array(marker_x) * arrow_size
    arrow_y = np.array(marker_y) * arrow_size
    for c in dkeys:
        arrow_z = np.array([c, c, c, c])
        ax.plot(arrow_x, arrow_y, arrow_z)


def two_plot(draws, xyMinMax, dim="2d"):
    """
    draws: [(data, is_feasibility), (data, is_feasibility)]
    dim: 2d or 3d
    """
    idx = 0
    if dim == "2d":
        fig, axs = plt.subplots(nrows=1, ncols=2)
        for data, isF in draws:
            scatter_2d(axs[idx], data, xyMinMax, is_feasibility=isF)
            idx += 1
        fig.tight_layout()
    else:
        fig = plt.figure()
        axs = [
            fig.add_subplot(121, projection="3d"),
            fig.add_subplot(122, projection="3d"),
        ]
        for data, isF in draws:
            scatter_3d(axs[idx], data, xyMinMax, is_feasibility=isF)
            idx += 1
    plt.show()
    return plt
