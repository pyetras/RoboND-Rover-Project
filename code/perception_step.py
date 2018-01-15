import perception
import numpy as np
import viz

def update_worldmap(rover):
  img = rover.img
  x, y = np.int_(rover.pos)
  rover.worldmap[x, y, :] = 255
  
  warped, masked = perception.perspect_transform(img)
  terrain = perception.color_thresh(warped)
  obstacles = masked - terrain

  # Calculate pixel values in rover-centric coords and distance/angle to all pixels
  xpix, ypix = np.int_(perception.rover_coords(terrain))
  obs_xpix, obs_ypix = np.int_(perception.rover_coords(obstacles))
  conf = perception.threshed_confidence(xpix, ypix)
  conf_obs = perception.threshed_confidence(obs_xpix, obs_ypix)
  xpix, ypix = perception.pix_to_world(xpix, ypix, x, y, rover.yaw, 200, 10)
  obs_xpix, obs_ypix = perception.pix_to_world(obs_xpix, obs_ypix, x, y, rover.yaw, 200, 10)

  if (rover.roll <= 2.0 or rover.roll >= 358.0) and \
      (rover.pitch <= 2.0 or rover.pitch >= 358.0):
    rover.worldmap[xpix, ypix, 2] += conf*80
    rover.worldmap[obs_xpix, obs_ypix, 0] += conf_obs*1
  
  rover.terrain = terrain
  rover.obstacles = obstacles
  rover.mask = masked

def perception_step(rover):
  update_worldmap(rover)
  warped, _ = perception.perspect_transform(rover.img)
  terrain = perception.color_thresh(warped)
  xpix, ypix = np.int_(perception.rover_coords(terrain))
  _, angles = perception.to_polar_coords(xpix, ypix)
  
  rover.mean_dir = np.clip(perception.biased_mean(angles), 
                           -np.pi / 12, # 15dg
                           np.pi / 12)
  
  # Vievport obstructed just in front of the rover
  rover.nav_angles = perception.unobstruction(rover, 0, np.pi / 6, 30)

  unobstructed = perception.unobstruction(rover, rover.mean_dir)
  rover.max_vel = np.clip(unobstructed * 4 + 2, 2, 6)
  rover.throttle_set = rover.max_vel / 10
  rover.vision_image = viz.draw_warped(rover)
  return rover