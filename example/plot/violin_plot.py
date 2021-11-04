import numpy as np
import pandas as pd
from matplotlib import pyplot as plt
import seaborn as sns


# old90, new90, old180, new180
dataframe = pd.read_csv("jkros_data_pandas.csv")

sns.set_style('whitegrid')
palette = sns.color_palette("pastel")
ax = sns.violinplot(x="Time (s)", y="Cases", data=dataframe, palette=palette, scale="count", inner=None)



# tips = sns.load_dataset('tips')
# print(tips)
# print(type(tips))

# ax = sns.violinplot(data=tips, y="day", x="total_bill", palette=palette, scale="count", inner=None)

for violin in ax.collections:
    bbox = violin.get_paths()[0].get_extents()
    x0, y0, width, height = bbox.bounds
    violin.set_clip_path(plt.Rectangle((x0, y0), width, height / 2, transform=ax.transData))

# sns.boxplot(y="day", x="total_bill", data=tips, saturation=1, showfliers=False,
#             width=0.3, boxprops={'zorder': 3, 'facecolor': 'none'}, ax=ax)
# old_len_collections = len(ax.collections)
# sns.stripplot(y="day", x="total_bill", data=tips, color='dodgerblue', ax=ax)
# for dots in ax.collections[old_len_collections:]:
#     dots.set_offsets(dots.get_offsets() + np.array([0, 0.12]))
ax.set_xlim(ax.get_xlim())
ax.set_ylim(ax.get_ylim())
plt.show()