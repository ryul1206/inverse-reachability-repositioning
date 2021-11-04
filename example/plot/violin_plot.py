import numpy as np
import pandas as pd
from matplotlib import pyplot as plt
import seaborn as sns


# old90, new90, old180, new180
dataframe = pd.read_csv("jkros_data_pandas.csv")

sns.set_style('whitegrid')
palette = sns.color_palette("pastel")

fig = plt.figure(num=1, figsize=(5, 3))
# Violin
# ax = sns.violinplot(x="Time (s)", y="Cases", data=dataframe, palette=palette, scale="count", inner=None)
# for violin in ax.collections:
#     bbox = violin.get_paths()[0].get_extents()
#     x0, y0, width, height = bbox.bounds
#     violin.set_clip_path(plt.Rectangle((x0, y0), width, height / 2, transform=ax.transData))

# Box
ax = sns.boxplot(
    x="Time (s)",
    y="Cases",
    data=dataframe,
    # palette=palette,
    # saturation=1,
    showfliers=False,
    # fliersize=2,
    # width=.4,
    width=.6,
    boxprops={'zorder': 3, 'facecolor': 'none'},
)

# plt.xscale("log")

plt.xticks(np.arange(0, 2.1, 0.2))
plt.ylabel("")
fs = 12
plt.xticks(fontsize=fs)
plt.yticks(fontsize=fs)
plt.xlabel("Time (s)", fontsize=fs)

fig.tight_layout()

ax.set_xlim(ax.get_xlim())
ax.set_ylim(ax.get_ylim())
plt.show()