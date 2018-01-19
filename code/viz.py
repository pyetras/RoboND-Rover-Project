import perception
import numpy as np
import cv2

def draw_warped(rover):
  terrain_conf = perception.conf_img(rover.terrain)
  obstacles_conf = perception.conf_img(rover.obstacles)
  w, h = rover.terrain.shape
  img = np.zeros((w, h, 3))
  img[:, :, 2]= terrain_conf * 255
  img[:, :, 0] = obstacles_conf * 255
  img[:, :, 1] = rover.rock * 255
  return img

def draw_worldmap(Rover):
    # Create a scaled map for plotting and clean up obs/nav pixels a bit
  if np.max(Rover.worldmap[:,:,2]) > 0:
    nav_pix = Rover.worldmap[:,:,2] > 0
    navigable = Rover.worldmap[:,:,2] * (255 / np.mean(Rover.worldmap[nav_pix, 2]))
  else:
    navigable = Rover.worldmap[:,:,2]
  if np.max(Rover.worldmap[:,:,0]) > 0:
    obs_pix = Rover.worldmap[:,:,0] > 0
    obstacle = Rover.worldmap[:,:,0] * (255 / np.mean(Rover.worldmap[obs_pix, 0]))
  else:
    obstacle = Rover.worldmap[:,:,0]

  likely_nav = navigable >= obstacle
  obstacle[likely_nav] = 0
  # navigable[np.invert(likely_nav)] = 0
  plotmap = np.zeros_like(Rover.worldmap)
  plotmap[:, :, 0] = obstacle
  plotmap[:, :, 2] = navigable
  plotmap = plotmap.clip(0, 255)
  # Overlay obstacle and navigable terrain map with ground truth map
  map_add = cv2.addWeighted(plotmap, 1, Rover.ground_truth, 0.5, 0)

  # Check whether any rock detections are present in worldmap
  rock_world_pos = Rover.worldmap[:,:,1].nonzero()
  # If there are, we'll step through the known sample positions
  # to confirm whether detections are real
  samples_located = 0
  if rock_world_pos[0].any():
    rock_size = 2
    for idx in range(len(Rover.samples_pos[0])):
      test_rock_x = Rover.samples_pos[0][idx]
      test_rock_y = Rover.samples_pos[1][idx]
      # print(test_rock_x, test_rock_y)
      rock_sample_dists = np.sqrt((test_rock_x - rock_world_pos[1])**2 + \
                (test_rock_y - rock_world_pos[0])**2)
      # print(rock_sample_dists)
      # If rocks were detected within 3 meters of known sample positions
      # consider it a success and plot the location of the known
      # sample on the map
      if np.min(rock_sample_dists) < 3:
        samples_located += 1
        map_add[test_rock_y-rock_size:test_rock_y+rock_size,
        test_rock_x-rock_size:test_rock_x+rock_size, :] = 255

  # Calculate some statistics on the map results
  # First get the total number of pixels in the navigable terrain map
  tot_nav_pix = np.float(len((plotmap[:,:,2].nonzero()[0])))
  # Next figure out how many of those correspond to ground truth pixels
  good_nav_pix = np.float(len(((plotmap[:,:,2] > 0) & (Rover.ground_truth[:,:,1] > 0)).nonzero()[0]))
  # Next find how many do not correspond to ground truth pixels
  bad_nav_pix = np.float(len(((plotmap[:,:,2] > 0) & (Rover.ground_truth[:,:,1] == 0)).nonzero()[0]))
  # Grab the total number of map pixels
  tot_map_pix = np.float(len((Rover.ground_truth[:,:,1].nonzero()[0])))
  # Calculate the percentage of ground truth map that has been successfully found
  perc_mapped = round(100*good_nav_pix/tot_map_pix, 1)
  # Calculate the number of good map pixel detections divided by total pixels
  # found to be navigable terrain
  if tot_nav_pix > 0:
    fidelity = round(100*good_nav_pix/(tot_nav_pix), 1)
  else:
    fidelity = 0
  # Flip the map for plotting so that the y-axis points upward in the display
#   map_add = np.flipud(map_add).astype(np.float32)
  # Add some text about map and rock sample detection results
  cv2.putText(map_add,"Time: "+str(np.round(Rover.total_time, 1))+' s', (0, 10),
        cv2.FONT_HERSHEY_COMPLEX, 0.4, (255, 255, 255), 1)
  cv2.putText(map_add,"Mapped: "+str(perc_mapped)+'%', (0, 25),
        cv2.FONT_HERSHEY_COMPLEX, 0.4, (255, 255, 255), 1)
  cv2.putText(map_add,"Fidelity: "+str(fidelity)+'%', (0, 40),
        cv2.FONT_HERSHEY_COMPLEX, 0.4, (255, 255, 255), 1)
  cv2.putText(map_add,"Rocks", (0, 55),
        cv2.FONT_HERSHEY_COMPLEX, 0.4, (255, 255, 255), 1)
  cv2.putText(map_add,"  Located: "+str(samples_located), (0, 70),
        cv2.FONT_HERSHEY_COMPLEX, 0.4, (255, 255, 255), 1)
  cv2.putText(map_add,"  Collected: "+str(Rover.samples_collected), (0, 85),
        cv2.FONT_HERSHEY_COMPLEX, 0.4, (255, 255, 255), 1)

  return map_add