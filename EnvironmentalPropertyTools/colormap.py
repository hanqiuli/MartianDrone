from matplotlib.image import imread
from matplotlib.colors import LinearSegmentedColormap
def colormap():
  img = imread("EnvironmentalPropertyTools/colormap.png")
  # img is 30 x 280 but we need just one col
  colors_from_img = img[:, 30, :]
  # commonly cmpas have 256 entries, but since img is 280 px => N=280
  my_cmap = LinearSegmentedColormap.from_list('my_cmap', colors_from_img, N=40)
  my_cmap = my_cmap.reversed()
  return my_cmap