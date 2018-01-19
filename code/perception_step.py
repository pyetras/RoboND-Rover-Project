import perception
import numpy as np
import viz
import time

def find_rock(rover, rock, rock_warped):
  x, y = rover.pos
  if len(rock.nonzero()[0]) > 15:
    rxy = perception.rover_coords(rock_warped)
    dist, angl = perception.to_polar_coords(*rxy)
    rxy = perception.pix_to_world(*rxy, x, y, rover.yaw, 200, 10)
    if len(angl) > 0:
      # print("Found a rock!!!", len(rock_warped.nonzero()[0]), np.std(angl))
      # print(np.mean(angl))
      ix = np.argmin(dist)
      rx, ry = rxy[0][ix], rxy[1][ix]
      rover.worldmap[rx, ry, 1] = 255
      # print([rx, ry][::-1], [] if rover.samples_pos is None else rover.samples_pos)
      rover.rock_dir = np.mean(angl)
      rover.rock_spotted = time.time()
      return

  if rover.rock_spotted is None or rover.rock_spotted < time.time() - 0.5:
    rover.rock_spotted = None
    rover.rock_dir = None

def update_worldmap(rover):
  img = rover.img
  x, y = rover.pos
  # rover.worldmap[x, y, :] = 255

  warped, masked = perception.perspect_transform(img)
  terrain = perception.color_thresh(warped)
  obstacles = masked - terrain
  rock = perception.rock_thresh(img)
  rock_warped = perception.rock_warped(rock)

  # Calculate pixel values in rover-centric coords and distance/angle to all pixels
  xpix, ypix = np.int_(perception.rover_coords(terrain))
  obs_xpix, obs_ypix = np.int_(perception.rover_coords(obstacles))
  conf = perception.threshed_confidence(xpix, ypix)
  conf_obs = perception.threshed_confidence(obs_xpix, obs_ypix)
  xpix, ypix = perception.pix_to_world(xpix, ypix, x, y, rover.yaw, 200, 10)
  obs_xpix, obs_ypix = perception.pix_to_world(obs_xpix, obs_ypix, x, y, rover.yaw, 200, 10)

  find_rock(rover, rock, rock_warped)

  if (rover.roll <= 1.0 or rover.roll >= 359.0) and \
      (rover.pitch <= 1.0 or rover.pitch >= 359.0):
    rover.worldmap[xpix, ypix, 2] += conf*80
    rover.worldmap[obs_xpix, obs_ypix, 0] += conf_obs*1

  rover.terrain = terrain
  rover.obstacles = obstacles
  rover.mask = masked
  rover.rock = rock_warped

def perception_step(rover):
  update_worldmap(rover)
  warped, _ = perception.perspect_transform(rover.img)
  terrain = perception.color_thresh(warped)
  xpix, ypix = np.int_(perception.rover_coords(terrain))
  dist, angles = perception.to_polar_coords(xpix, ypix)
  angles = angles[dist < 100]

  biased_mean_quantile = np.clip(
    (np.std(angles) / rover.max_std) * 0.25 + 0.55, 0.55, 0.8)
  biased_mean_quantile_right = np.clip(
    (np.std(angles) / rover.max_std) * -0.25 - 0.55, -0.8, -0.55)

  rover.mean_dir = np.clip(perception.biased_mean(angles, biased_mean_quantile),
                           -rover.max_angle,
                           rover.max_angle)
  mean_dir_right = np.clip(perception.biased_mean(angles, biased_mean_quantile_right),
                           -rover.max_angle,
                           rover.max_angle)

  # Viewport obstructed just in front of the rover
  rover.nav_angles = perception.unobstruction(rover, 0, np.pi / 8, 15)

  x, y = rover.start_pos[0] - rover.pos[0], rover.start_pos[1] - rover.pos[1]
  _, rover.home_dir = perception.to_polar_coords(np.array([x]), np.array([y]))
  yaw = rover.yaw*np.pi/180
  if yaw > np.pi:
    yaw -= 2*np.pi
  rover.home_dir = rover.home_dir[0] - yaw
  if len(angles) > 0:
    rover.home_mean_dir = np.clip(rover.home_dir,
        mean_dir_right, rover.mean_dir)

  return rover
