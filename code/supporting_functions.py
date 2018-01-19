import numpy as np
import cv2
from PIL import Image
from io import BytesIO, StringIO
import base64
import time
import viz

# Define a function to convert telemetry strings to float independent of decimal convention
def convert_to_float(string_to_convert):
    if ',' in string_to_convert:
      float_value = np.float(string_to_convert.replace(',','.'))
    else:
      float_value = np.float(string_to_convert)
    return float_value

def update_rover(Rover, data):
    # Initialize start time and sample positions
    if Rover.start_time == None:
      Rover.start_time = time.time()
      Rover.total_time = 0
      samples_xpos = np.int_([convert_to_float(pos.strip()) for pos in data["samples_x"].split(';')])
      samples_ypos = np.int_([convert_to_float(pos.strip()) for pos in data["samples_y"].split(';')])
      Rover.samples_pos = [samples_xpos, samples_ypos]
      Rover.samples_to_find = np.int(data["sample_count"])
    # Or just update elapsed time
    else:
      tot_time = time.time() - Rover.start_time
      if np.isfinite(tot_time):
          Rover.total_time = tot_time
    # Print out the fields in the telemetry data dictionary
    # print(data.keys())
    # The current speed of the rover in m/s
    Rover.vel = convert_to_float(data["speed"])
    # The current position of the rover
    Rover.pos = [convert_to_float(pos.strip()) for pos in data["position"].split(';')]
    if Rover.start_pos is None:
        Rover.start_pos = Rover.pos
    # The current yaw angle of the rover
    Rover.yaw = convert_to_float(data["yaw"])
    # The current yaw angle of the rover
    Rover.pitch = convert_to_float(data["pitch"])
    # The current yaw angle of the rover
    Rover.roll = convert_to_float(data["roll"])
    # The current throttle setting
    Rover.throttle = convert_to_float(data["throttle"])
    # The current steering angle
    Rover.steer = convert_to_float(data["steering_angle"])
    # Near sample flag
    Rover.near_sample = np.int(data["near_sample"])
    # Picking up flag
    Rover.picking_up = np.int(data["picking_up"])
    # Update number of rocks collected
    Rover.samples_collected = Rover.samples_to_find - np.int(data["sample_count"])

    # print('speed =',Rover.vel, 'position =', Rover.pos, 'throttle =',
    # Rover.throttle, 'steer_angle =', Rover.steer, 'near_sample:', Rover.near_sample,
    # 'picking_up:', data["picking_up"], 'sending pickup:', Rover.send_pickup,
    # 'total time:', Rover.total_time, 'samples remaining:', data["sample_count"],
    # 'samples collected:', Rover.samples_collected)
    # Get the current image from the center camera of the rover
    imgString = data["image"]
    image = Image.open(BytesIO(base64.b64decode(imgString)))
    Rover.img = np.asarray(image)

    # Return updated Rover and separate image for optional saving
    return Rover, image

# Define a function to create display output given worldmap results
def create_output_images(Rover):
    # Convert map and vision image to base64 strings for sending to server
    pil_img = Image.fromarray(viz.draw_worldmap(Rover).astype(np.uint8))
    buff = BytesIO()
    pil_img.save(buff, format="JPEG")
    encoded_string1 = base64.b64encode(buff.getvalue()).decode("utf-8")

    pil_img = Image.fromarray(viz.draw_warped(Rover).astype(np.uint8))
    buff = BytesIO()
    pil_img.save(buff, format="JPEG")
    encoded_string2 = base64.b64encode(buff.getvalue()).decode("utf-8")

    return encoded_string1, encoded_string2



