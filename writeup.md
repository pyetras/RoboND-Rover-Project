# Project: Search and Sample Return

[//]: # (Image References)

[image1]: ./misc/rover_image.jpg
[image2]: ./calibration_images/example_grid1.jpg
[image3]: ./calibration_images/example_rock1.jpg
[fsm]: ./misc/D0E3C5A3-D4F0-4207-9E97-7DF045D5C0C4.jpeg
[unobstruction]: ./misc/unobstruction.png
[rock]: ./misc/rock.png
[moviepy]: ./misc/moviepy.png
[fail]: ./misc/fail.png

## [Rubric](https://review.udacity.com/#!/rubrics/916/view) Points
 Here I will consider the rubric points individually and describe how I addressed each point in my implementation.

### Notebook Analysis
#### 1. Run the functions provided in the notebook on test images (first with the test data provided, next on data you have recorded). Add/modify functions to allow for color selection of obstacles and rock samples.
For obstacle detection I've used the algorithm from walkthrough video - all pixels that are not terrain are considered obstacles.

For rock detection I've selected pixels with color between (130, 110, 0) and (250, 220, 60) as rocks. Additionally I've rejected all pixels that don't have 4 neighbors also considered as rocks - this removed much of the noise and outliers that were often present before I had the color range right.

```
def rock_thresh(img):
  im = cv2.inRange(img, (130, 110, 0), (250, 220, 60))
  im[im.nonzero()] = 1
  nudged = np.copy(im)
  for (shift, axis) in [(1, 0), (1, 1), (-1, 0), (-1, 1)]:
    nudged += np.roll(im, shift, axis)
  im[nudged < 5] = 0
  im[nudged >= 5] = 1
  return im
```
![rock][rock]
Before and after outlier elimination.

#### 1. Populate the `process_image()` function with the appropriate analysis steps to map pixels identifying navigable terrain, obstacles and rock samples into a worldmap.  Run `process_image()` on your test data using the `moviepy` functions provided to create video output of your result.

I've modified `process_image()` to give me the same output as the rover program. The data is read from `RoverState` but updated from the csv file instead of the websocket. Additionally I'm displaying the rock threshold mask.

[![video][moviepy]](./output/test_mapping.mp4)

### Autonomous Navigation and Mapping

#### 1. Fill in the `perception_step()` (at the bottom of the `perception.py` script) and `decision_step()` (in `decision.py`) functions in the autonomous mapping scripts and an explanation is provided in the writeup of how and why these functions were modified as they were.

`perception.py` was forked into two files, `perception.py` and `perception_step.py`, the latter computes additional rover properties using functions from the former. `perception.py` is the same as the perception functions in the notebook.

Computed properties are as follows:

* `rover.mean_dir`: the name is completely wrong, it is the angle at the `k`% of angles of visible terrain pixels sorted from smallest to largest. If `k=0.5` median angle is selected, for `k=1` it's the maximum etc. If `k` points to an angle larger than the median angle (in the middle), the rover following that angle should prefer to "hug" the left wall. `k` is between 0.55 and 0.70, scaled according to the standard deviation of the angles. Narrower corridors get smaller `k`, so the rover drives more straight.

* `rover.home_dir`: rover-centric angle between current and starting position.

* `rover.home_mean_dir`: a combination of previous two, it's `home_dir` clipped to: the wall following angle of `mean_dir` and a right-wall-following angle computed with `k2 = 1 - k`. It's meant to drive the rover straight if home position is somewhere ahead, or close to one of the walls if it's not in the current view.

* Rock detection and `rover.rock_dir`: rocks are detected using a `rock_threshold` function and perspective and coordinate transformations as for terrain and obstacles. `rock_dir` remains valid for 0.5 second after it's no more visible to prevent "jitter" when approaching the rock.

These properties are computed only based on the vision pixels that are no further than 10m from the rover. I've found this to give more accurate results.

`decision.py` implements the following state machine:

![alt text][fsm]

The rover starts in `start` state, which will drive it forward until first obstacle is met. This is to prevent the situation where the rover would drive in circles, because it starts too far from any walls.

Next, the main loop of the program begins in state `forward`. The rover navigates according to `rover.mean_dir`, scaling the speed according to `unobstruction` parameter.

![unobstruction][unobstruction]

Unobstruction is the ratio of terrain pixels to the obstacle pixels in the small area along the movement vector (green in the picture). The more terrain pixels, the faster the rover can go.

 The rover will transition to `brake` when the ratio of navigable pixels within short distance in front of it is too low (computed similarly as unobstruction, but along a 0dg navigation vector, so just right in front of the rover). `brake` will stop the rover and turn until the ratio is raised again.

`forward` will transition to `brake_rock` whenever a `rover.rock_dir` is available and pointing to the left side of the rover or up to 30 degrees in the right-hand-side direction. This is to prevent the rover from losing it's original direction and skipping a part of the map.

`brake_rock` will stop the rover and wait until pitch and roll stabilise. If the rover was going too fast, it's possible that it will overshoot the rock. Therefore, if the rock is not visible after stopping the rover will attempt to back up. If the rock is visible, `turn_rock` will face it's direction and `forward_rock` will approach it. At any given time, if the rock is not visible for more than 0.5 second, the rover transitions back to `forward`.

In any state of the machine, if the rover is near rock, it will fully stop and pick up the rock. Only then normal operation resumes.

Whenever the rover picks up the last rock, machine will transition to `turn_home`. This state rotates the rover in the direction of starting position `rover.home_dir`. The next state is `forward_home` which acts as `forward`, except it will transition back to `turn_home` every 30s, use `rover.home_mean_dir` for navigation, and terminate when the distance to starting position is less or equal 5m.

All of the "movement" states (highlighted in red in the diagram) can additionally transition into the `unstuck` state. We determine the rover to be stuck, if the total distance traveled within last 3s is less than 0.5m (this is computed only when the rover is continuously in "movement" states). To unstuck, we stop, back up the rover and turn. Then the rover proceeds back to `forward` or `forward_home` state.

#### 2. Launching in autonomous mode your rover can navigate and map autonomously.  Explain your results and how you might improve them in your writeup.

The following results were achieved on Linux with fantastic settings, 15-20 FPS and with one additional package installed (`pip install absl-py`, alternatively the flag code may be removed from `drive_rover.py` to run without it).

I've implemented the project with all extensions: picking up and returning the samples to base. I've been able to succesfully run it almost every time, with the exception of when the rocks are spawned next to each other, which might sometime confuse the rover. To solve this a "true" centroid algorithm could be implemented for processing the rock image, to filter out just one rock. Alternatively, standard deviation of rock angles could be used to determine if the detected blob is small enough.

A successful run should take less than 10 mins, with 98-99% mapped and fidelity above 0.65.

I've tried to increase fidelity by applying "uncertainty" - pixels further away from the rover or more to the side are assigned lower probability. This has not improved my solution, perhaps a formal verification of the assumptions I made regarding the probability against perspectTransform algorithm or tweaking the observation concatenation formula would yield a better result.

![fail][fail]

For a more complicated map it would be interesting to see a fully fledged navigation algorithm taking into account the computed worldmap, dividing it into sections and purposefully navigating to visit them all. For this map, however, the wall-following algorithm proved absolutely sufficient.

## Video

[![video](https://img.youtube.com/vi/IUhwW2wrBYU/0.jpg)](https://www.youtube.com/watch?v=IUhwW2wrBYU)

