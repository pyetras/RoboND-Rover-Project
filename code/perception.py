import numpy as np
import cv2

# Define calibration box in source (actual) and destination (desired) coordinates
# These source and destination points are defined to warp the image
# to a grid where each 10x10 pixel square represents 1 square meter
# The destination box will be 2*dst_size on each side
dst_size = 5
# Set a bottom offset to account for the fact that the bottom of the image
# is not the position of the rover but a bit in front of it
# this is just a rough guess, feel free to change it!
bottom_offset = 6
image = np.zeros((160, 320))
_source = np.float32([[14, 140], [301 ,140],[200, 96], [118, 96]])
_destination = np.float32([[image.shape[1]/2 - dst_size, image.shape[0] - bottom_offset],
            [image.shape[1]/2 + dst_size, image.shape[0] - bottom_offset],
            [image.shape[1]/2 + dst_size, image.shape[0] - 2*dst_size - bottom_offset],
            [image.shape[1]/2 - dst_size, image.shape[0] - 2*dst_size - bottom_offset],
            ])

# Define a function to perform a perspective transform
# I've used the example grid image above to choose source points for the
# grid cell in front of the rover (each grid cell is 1 square meter in the sim)
# Define a function to perform a perspective transform
def perspect_transform(img):
  M = cv2.getPerspectiveTransform(_source, _destination)
  warped = cv2.warpPerspective(img, M, (img.shape[1], img.shape[0]))# keep same size as input image
  mask = cv2.warpPerspective(np.ones_like(img[:,:,0]), M, (img.shape[1], img.shape[0]))# keep same size as input image
  return warped, mask

# Identify pixels above the threshold
# Threshold of RGB > 160 does a nice job of identifying ground pixels only
def color_thresh(img, rgb_thresh=(160, 160, 160)):
  # Create an array of zeros same xy size as img, but single channel
  color_select = np.zeros_like(img[:,:,0])
  # Require that each pixel be above all three threshold values in RGB
  # above_thresh will now contain a boolean array with "True"
  # where threshold was met
  above_thresh = (img[:,:,0] > rgb_thresh[0]) \
                  & (img[:,:,1] > rgb_thresh[1]) \
                  & (img[:,:,2] > rgb_thresh[2])
  # Index the array of zeros with the boolean array and set to 1
  color_select[above_thresh] = 1
  # Return the binary image
  return color_select

# Define a function to convert from image coords to rover coords
def rover_coords(binary_img):
  # Identify nonzero pixels
  ypos, xpos = binary_img.nonzero()
  # Calculate pixel positions with reference to the rover position being at the
  # center bottom of the image.
  x_pixel = -(ypos - binary_img.shape[0]).astype(np.float)
  y_pixel = -(xpos - binary_img.shape[1]/2 ).astype(np.float)
  return x_pixel, y_pixel

# Define a function to convert to radial coords in rover space
def to_polar_coords(x_pixel, y_pixel):
  # Convert (x_pixel, y_pixel) to (distance, angle)
  # in polar coordinates in rover space
  # Calculate distance to each pixel
  dist = np.sqrt(x_pixel**2 + y_pixel**2)
  # Calculate angle away from vertical for each pixel
  angles = np.arctan2(y_pixel, x_pixel)
  return dist, angles

# Define a function to map rover space pixels to world space
def rotate_pix(xpix, ypix, yaw):
  # Convert yaw to radians
  yaw_rad = yaw * np.pi / 180
  xpix_rotated = (xpix * np.cos(yaw_rad)) - (ypix * np.sin(yaw_rad))
  ypix_rotated = (xpix * np.sin(yaw_rad)) + (ypix * np.cos(yaw_rad))
  # Return the result
  return xpix_rotated, ypix_rotated

def translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale):
  # Apply a scaling and a translation
  xpix_translated = (xpix_rot / scale) + xpos
  ypix_translated = (ypix_rot / scale) + ypos
  # Return the result
  return xpix_translated, ypix_translated

# Define a function to apply rotation and translation (and clipping)
# Once you define the two functions above this function should work
def pix_to_world(xpix, ypix, xpos, ypos, yaw, world_size, scale):
  # Apply rotation
  xpix_rot, ypix_rot = rotate_pix(xpix, ypix, yaw)
  # Apply translation
  xpix_tran, ypix_tran = translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale)
  # Perform rotation, translation and clipping all at once
  x_pix_world = np.clip(np.int_(xpix_tran), 0, world_size - 1)
  y_pix_world = np.clip(np.int_(ypix_tran), 0, world_size - 1)
  # Return the result
  return x_pix_world, y_pix_world

def gaussian(x, mu, sig):
  return np.exp(-np.power(x - mu, 2.) / (2 * np.power(sig, 2.)))

def threshed_confidence(xpix, ypix, max_dist = 200):
  mu = 0.8
  sig = 0

  dist, angle = to_polar_coords(xpix, ypix)
  angle_prob = gaussian(angle, sig, mu) / gaussian(sig, sig, mu)
  dist = -dist / max_dist
  dist_prob = np.clip(np.tanh(dist * 7 + 2) / 2.0 + 0.5, 0, 1)
  return dist_prob * np.abs(angle_prob)

def conf_img(threshed):
  xpix, ypix = rover_coords(threshed)
  conf = np.zeros_like(threshed, dtype=np.float)
  conf2 = threshed_confidence(xpix, ypix)
  conf[np.int_(xpix) - 1, np.int_(ypix)-160] = conf2
  return np.fliplr(np.flipud(conf))

def biased_mean(angles):
  if len(angles) <= 1: return 0
  angles = sorted(angles)
  return angles[len(angles)*2//3]
