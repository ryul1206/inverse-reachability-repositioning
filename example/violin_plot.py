from matplotlib import pyplot as plt
import seaborn as sns
import numpy as np

sns.set_style('white')
tips = sns.load_dataset('tips')
palette = sns.cubehelix_palette(start=.5, rot=-.5, dark=0.3, light=0.7)
ax = sns.violinplot(y="day", x="total_bill", data=tips,
                    palette=palette,
                    scale="width", inner=None)
xlim = ax.get_xlim()
ylim = ax.get_ylim()
for violin in ax.collections:
    bbox = violin.get_paths()[0].get_extents()
    x0, y0, width, height = bbox.bounds
    violin.set_clip_path(plt.Rectangle((x0, y0), width, height / 2, transform=ax.transData))

sns.boxplot(y="day", x="total_bill", data=tips, saturation=1, showfliers=False,
            width=0.3, boxprops={'zorder': 3, 'facecolor': 'none'}, ax=ax)
old_len_collections = len(ax.collections)
sns.stripplot(y="day", x="total_bill", data=tips, color='dodgerblue', ax=ax)
for dots in ax.collections[old_len_collections:]:
    dots.set_offsets(dots.get_offsets() + np.array([0, 0.12]))
ax.set_xlim(xlim)
ax.set_ylim(ylim)
plt.show()