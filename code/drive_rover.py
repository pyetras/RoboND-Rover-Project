# Do the necessary imports
import shutil
import base64
from datetime import datetime
import os
import cv2
import numpy as np
import socketio
import eventlet
import eventlet.wsgi
from PIL import Image
from flask import Flask
from io import BytesIO, StringIO
import json
import pickle
import matplotlib.image as mpimg
import time
from absl import flags, app
import logging
logging.getLogger('werkzeug').setLevel(logging.ERROR)
logging.getLogger('socketio').setLevel(logging.ERROR)
logging.getLogger('engineio').setLevel(logging.ERROR)

FLAGS = flags.FLAGS

flags.DEFINE_string('image_folder', '', 'Path to image folder. This is where the images from the run will be saved.')

# Import functions for perception and decision making
from perception_step import perception_step
from decision import decision_step
from supporting_functions import update_rover, create_output_images
from rover_state import RoverState
# Initialize socketio server and Flask application
# (learn more at: https://python-socketio.readthedocs.io/en/latest/)
sio = socketio.Server(logger = False)

# Initialize our rover
Rover = RoverState()

# Variables to track frames per second (FPS)
# Initialize frame counter
frame_counter = 0
# Initialize second counter
second_counter = time.time()
fps = None


# Define telemetry function for what to do with incoming data
@sio.on('telemetry')
def telemetry(sid, data):

  global frame_counter, second_counter, fps
  frame_counter+=1
  # Do a rough calculation of frames per second (FPS)
  if (time.time() - second_counter) > 1:
    fps = frame_counter
    frame_counter = 0
    second_counter = time.time()
  print("Current FPS: {}".format(fps))

  if data:
    global Rover
    # Initialize / update Rover with current telemetry
    Rover, image = update_rover(Rover, data)

    if np.isfinite(Rover.vel):

      # Execute the perception and decision steps to update the Rover's state
      Rover = perception_step(Rover)
      Rover = decision_step(Rover)

      # Create output images to send to server
      out_image_string1, out_image_string2 = create_output_images(Rover)

      # The action step!  Send commands to the rover!

      # Don't send both of these, they both trigger the simulator
      # to send back new telemetry so we must only send one
      # back in respose to the current telemetry data.

      # If in a state where want to pickup a rock send pickup command
      if Rover.send_pickup and not Rover.picking_up:
        send_pickup()
        # Reset Rover flags
        Rover.send_pickup = False
      else:
        # Send commands to the rover!
        commands = (Rover.throttle, Rover.brake, Rover.steer)
        send_control(commands, out_image_string1, out_image_string2)

    # In case of invalid telemetry, send null commands
    else:

      # Send zeros for throttle, brake and steer and empty images
      send_control((0, 0, 0), '', '')

    # If you want to save camera images from autonomous driving specify a path
    # Example: $ python drive_rover.py image_folder_path
    # Conditional to save image frame if folder was specified
    if FLAGS.image_folder != '':
      timestamp = datetime.utcnow().strftime('%Y_%m_%d_%H_%M_%S_%f')[:-3]
      image_filename = os.path.join(FLAGS.image_folder, timestamp)
      image.save('{}.jpg'.format(image_filename))

  else:
    sio.emit('manual', data={}, skip_sid=True)

@sio.on('connect')
def connect(sid, environ):
  print("connect ", sid)
  send_control((0, 0, 0), '', '')
  sample_data = {}
  sio.emit(
    "get_samples",
    sample_data,
    skip_sid=True)

def send_control(commands, image_string1, image_string2):
  # Define commands to be sent to the rover
  data={
    'throttle': commands[0].__str__(),
    'brake': commands[1].__str__(),
    'steering_angle': commands[2].__str__(),
    'inset_image1': image_string1,
    'inset_image2': image_string2,
    }
  # Send commands via socketIO server
  sio.emit(
    "data",
    data,
    skip_sid=True)
  eventlet.sleep(0)
# Define a function to send the "pickup" command
def send_pickup():
  print("Picking up")
  pickup = {}
  sio.emit(
    "pickup",
    pickup,
    skip_sid=True)
  eventlet.sleep(0)

def main(argv):
  #os.system('rm -rf IMG_stream/*')
  if FLAGS.image_folder != '':
    print("Creating image folder at {}".format(FLAGS.image_folder))
    if not os.path.exists(FLAGS.image_folder):
      os.makedirs(FLAGS.image_folder)
    else:
      shutil.rmtree(FLAGS.image_folder)
      os.makedirs(FLAGS.image_folder)
    print("Recording this run ...")
  else:
    print("NOT recording this run ...")

  # wrap Flask application with socketio's middleware
  app = Flask(__name__)
  app = socketio.Middleware(sio, app)

  # deploy as an eventlet WSGI server
  eventlet.wsgi.server(eventlet.listen(('', 4567)), app)

if __name__ == '__main__':
  app.run(main)
