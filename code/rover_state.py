import matplotlib.image as mpimg
import numpy as np
from collections import deque

# Define RoverState() class to retain rover state parameters
class RoverState():
  def __init__(self):
    # Read in ground truth map and create 3-channel green version for overplotting
    # NOTE: images are read in by default with the origin (0, 0) in the upper left
    # and y-axis increasing downward.
    ground_truth = mpimg.imread('../calibration_images/map_bw.png')
    # This next line creates arrays of zeros in the red and blue channels
    # and puts the map into the green channel.  This is why the underlying
    # map output looks green in the display image
    ground_truth_3d = np.dstack((ground_truth*0, ground_truth*255, ground_truth*0)).astype(np.float)

    self.start_time = None # To record the start time of navigation
    self.total_time = None # To record total duration of naviagation
    self.img = None # Current camera image
    self.pos = None # Current position (x, y)
    self.start_pos = None
    self.yaw = None # Current yaw angle
    self.pitch = None # Current pitch angle
    self.roll = None # Current roll angle
    self.vel = None # Current velocity
    self.steer = 0 # Current steering angle
    self.throttle = 0 # Current throttle value
    self.brake = 0 # Current brake value
    self.nav_angles = None # Angles of navigable terrain pixels
    self.nav_dists = None # Distances of navigable terrain pixels
    self.ground_truth = ground_truth_3d # Ground truth worldmap
    self.mode = 'start' # Current mode (can be forward or stop)
    self.throttle_set = 0.2 # Throttle setting when accelerating
    self.brake_set = 10 # Brake setting when braking
    # The stop_forward and go_forward fields below represent total count
    # of navigable terrain pixels.  This is a very crude form of knowing
    # when you can keep going and when you should stop.  Feel free to
    # get creative in adding new fields or modifying these!
    self.stop_forward = 0.3 # Threshold to initiate stopping
    self.go_forward = 0.40 # Threshold to go forward again
    self.max_vel = 2 # Maximum velocity (meters/second)
    self.returning = False
    # Image output from perception step
    # Update this image to display your intermediate analysis steps
    # on screen in autonomous mode
    self.vision_image = np.zeros((160, 320, 3), dtype=np.float)
    # Worldmap
    # Update this image with the positions of navigable terrain
    # obstacles and rock samples
    self.worldmap = np.zeros((200, 200, 3), dtype=np.float)
    self.samples_pos = [[],[]] # To store the actual sample positions
    self.found_samples = []
    self.samples_to_find = 0 # To store the initial count of samples
    self.samples_located = 0 # To store number of samples located on map
    self.samples_collected = 0 # To count the number of samples collected
    self.near_sample = 0 # Will be set to telemetry value data["near_sample"]
    self.picking_up = 0 # Will be set to telemetry value data["picking_up"]
    self.send_pickup = False # Set to True to trigger rock pickup

    # stddev of angles in an empty map (no obstacles).
    self.max_std = 0.5877424739393784
    self.max_angle = np.pi * 15 / 180

    self.dist = 0 # for unstucking.
    self.history = deque()

    self.rock_spotted = None
    self.last_turn = None
