import perception
import numpy as np

def draw_warped(rover):
  terrain_conf = perception.conf_img(rover.terrain)
  obstacles_conf = perception.conf_img(rover.obstacles)
  w, h = rover.terrain.shape
  img = np.zeros((w, h, 3))
  img[:, :, 2]= terrain_conf * 255
  img[:, :, 0] = obstacles_conf * 255
  return img