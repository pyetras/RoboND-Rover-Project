import numpy as np
import perception
import time
from collections import deque

def forward_step(Rover):
  if Rover.rock_dir is not None and Rover.rock_dir + (30*np.pi/180) >= 0:
    return 'brake_rock'

  return drive(Rover, Rover.mean_dir) or 'forward'

def drive(Rover, direction):
  Rover.brake = 0
  unobstruction = perception.unobstruction(Rover, direction)
  max_vel = np.clip(unobstruction * 5 + 2, 2, 7)
  Rover.throttle_set = max_vel / 10
  if Rover.vel < max_vel:
    if Rover.vel < 0.6:
      Rover.throttle = 1
    else:
      # Set throttle value to throttle setting
      Rover.throttle = Rover.throttle_set
    Rover.brake = 0
  else: # Else coast
    Rover.throttle = 0
    Rover.brake = 1
  # Set steering to average angle clipped to the range +/- 15
  Rover.steer = np.clip(direction, -Rover.max_angle, Rover.max_angle)* 180/np.pi
  if Rover.nav_angles >= Rover.stop_forward:
    return None
  else:
    return 'brake'

def start_step(Rover):
  return drive(Rover, 0) or 'start'

def brake_step(Rover):
  Rover.steer = 0
  # If we're in stop mode but still moving keep braking
  if Rover.vel > 0.2:
    Rover.throttle = 0
    Rover.brake = Rover.brake_set
    Rover.steer = 0
  # If we're not moving (vel < 0.2) then do something else
  elif Rover.vel <= 0.2:
    # Now we're stopped and we have vision data to see if there's a path forward
    if Rover.nav_angles < Rover.go_forward:
      Rover.throttle = 0
      # Release the brake to allow turning
      Rover.brake = 0
      # Turn range is +/- 15 degrees, when stopped the next line will induce 4-wheel turning
      Rover.steer = -15 # Could be more clever here about which way to turn
    # If we're stopped but see sufficient navigable terrain in front then go!
    else:
      # Set throttle back to stored value
      Rover.throttle = Rover.throttle_set
      # Release the brake
      Rover.brake = 0
      # Set steer to mean angle
      Rover.steer = np.clip(Rover.mean_dir * 180/np.pi, -15, 15)
      return 'forward'
  return 'brake'

def forward_rock_step(Rover):
  if Rover.rock_dir is None:
    return 'forward'

  drive(Rover, Rover.rock_dir)
  return 'forward_rock'

def brake_rock_step(Rover):
  if Rover.vel > 0.2:
    Rover.throttle = 0
    Rover.brake = Rover.brake_set
    Rover.steer = 0
    return 'brake_rock'
  else:
    Rover.next_step = 'back_if_no_rock'
    Rover.sleep_time = time.time() + 2
    Rover.back_time = time.time() + 4
    return 'sleep'

def back_if_no_rock_step(Rover):
  if time.time() > Rover.back_time or Rover.rock_dir is not None:
    del Rover.back_time
    return 'turn_rock'
  else:
    Rover.throttle = -1
    return 'back_if_no_rock'

def turn_rock_step(Rover):
  if Rover.vel > 0.2:
    Rover.throttle = 0
    Rover.brake = Rover.brake_set
    Rover.steer = 0
    return 'turn_rock'

  Rover.brake = 0
  if Rover.rock_dir is None:
    return 'forward'

  if (np.abs(Rover.rock_dir) * 180.0 / np.pi) <= 10:
    return 'forward_rock'
  else:
    Rover.steer = np.clip(Rover.rock_dir * 180/np.pi, -15, 15)
    return 'turn_rock'

def select_action(Rover):
  if Rover.mode == 'forward':
    if Rover.returning:
      Rover.mode = forward_home_step(Rover)
    else:
      Rover.mode = forward_step(Rover)
  elif Rover.mode == 'brake':
    Rover.mode = brake_step(Rover)
  elif Rover.mode == 'forward_rock':
    Rover.mode = forward_rock_step(Rover)
  elif Rover.mode == 'brake_rock':
    Rover.mode = brake_rock_step(Rover)
  elif Rover.mode == 'turn_rock':
    Rover.mode = turn_rock_step(Rover)
  elif Rover.mode == 'sleep':
    Rover.mode = sleep_step(Rover)
  elif Rover.mode == 'unstuck':
    Rover.mode = unstuck_step(Rover)
  elif Rover.mode == 'unstuck_back':
    Rover.mode = unstuck_back_step(Rover)
  elif Rover.mode == 'unstuck_turn':
    Rover.mode = unstuck_turn_step(Rover)
  elif Rover.mode == 'turn_home':
    Rover.mode = turn_home_step(Rover)
  elif Rover.mode == 'start':
    Rover.mode = start_step(Rover)
  elif Rover.mode == 'end':
    Rover.mode = end(Rover)
  elif Rover.mode == 'back_if_no_rock':
    Rover.mode = back_if_no_rock_step(Rover)
  else:
    raise Rover.mode

def sleep_step(Rover):
  if time.time() >= Rover.sleep_time:
    Rover.throttle = 0
    Rover.steer = 0
    Rover.brake = 0
    nxt = Rover.next_step
    del Rover.next_step
    del Rover.sleep_time
    return nxt
  else:
    return 'sleep'

def unstuck_step(Rover):
  if Rover.vel > 0.2:
    Rover.throttle = 0
    Rover.brake = Rover.brake_set
    Rover.steer = 0
    return 'unstuck'
  Rover.back_time = time.time() + 2
  return 'unstuck_back'

def unstuck_back_step(Rover):
  if time.time() > Rover.back_time:
    del Rover.back_time
    return 'unstuck_turn'
  else:
    Rover.throttle = -0.3
    Rover.unstuck_time = time.time() + 1
    return 'unstuck_back'

def unstuck_turn_step(Rover):
  if time.time() > Rover.unstuck_time:
    del Rover.unstuck_time
    return 'forward'

  if np.abs(Rover.vel) > 0.2:
    Rover.throttle = 0
    Rover.brake = Rover.brake_set
    Rover.steer = 0
  else:
    Rover.steer = -15

  return 'unstuck_turn'

def turn_home_step(Rover):
  print(Rover.home_dir, Rover.yaw)
  if Rover.vel > 0.2:
    Rover.throttle = 0
    Rover.brake = Rover.brake_set
    Rover.steer = 0
    return 'turn_home'
  if np.abs(Rover.home_dir * 180 / np.pi) > 10:
    Rover.steer = -15
    return 'turn_home'

  Rover.last_turn = time.time()
  return 'forward'

def forward_home_step(Rover):
  print(Rover.home_mean_dir)
  x, y = Rover.pos
  hx, hy = Rover.start_pos

  if np.abs(Rover.home_dir) > np.pi/2: # Need to make a 90dg+ turn
    if Rover.last_turn + 30 < time.time():
      return 'turn_home'

  if np.sqrt((x-hx)**2 + (y-hy)**2) <= 5:
    Rover.brake = 10
    Rover.steer = 0
    Rover.throttle = 0
    return 'end'
  else:
    return drive(Rover, Rover.home_mean_dir) or 'forward'

def end(Rover):
  Rover.throttle = 0
  Rover.brake = Rover.brake_set
  Rover.steer = 0
  return 'end'

def decision_step(Rover):
  if Rover.near_sample and Rover.vel > 0:
    Rover.brake = 10
    return Rover

  # If in a state where want to pickup a rock send pickup command
  if Rover.near_sample and Rover.vel == 0 and not Rover.picking_up:
    Rover.send_pickup = True
    Rover.mode = 'forward'

  if Rover.picking_up:
    return Rover

  Rover.throttle = 0
  Rover.brake = 0
  Rover.steer = 0

  old_mode = Rover.mode
  select_action(Rover)
  if old_mode != Rover.mode:
    select_action(Rover)

  if Rover.mode in ['forward', 'forward_rock', 'start']:
    if len(Rover.history) > 0:
      (lastx, lasty), _= Rover.history[-1]
      x, y = Rover.pos
      Rover.dist += np.sqrt((x - lastx)**2 + (y - lasty)**2)

    Rover.history.append((Rover.pos, time.time()))

    while len(Rover.history) > 0:
      (lastx, lasty), tm = Rover.history[0]
      if tm > time.time() - 4.0:
        break
      Rover.history.popleft()
      (x, y), _ = Rover.history[0]
      Rover.dist -= np.sqrt((x - lastx)**2 + (y - lasty)**2)

    if Rover.history[0][1] <= time.time() - 3.0:
      if Rover.dist < 3.0*0.1:
        Rover.mode = 'unstuck'
  else:
    Rover.history = deque()
    Rover.dist = 0

  if Rover.samples_collected == 6 and not Rover.returning:
    Rover.mode = 'turn_home'
    Rover.returning = True

  print(Rover.rock_dir, Rover.returning, Rover.mode, Rover.dist)

  return Rover

