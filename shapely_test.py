from shapely.geometry import LineString, Polygon
import numpy as np
from shapely.geometry import Point, MultiPoint
from math import log
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from mpl_toolkits.mplot3d import Axes3D

line = MultiPoint([(0, 0, -10), (0, 0, 10)])
polygon = Polygon([(-2, -2, 0 ), (-2, 2, 0), (2, -2, 0), (2,2,0)])

if line.intersection(polygon):
    print("je")