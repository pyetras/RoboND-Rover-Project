## Project: Search and Sample Return
### Writeup Template: You can use this file as a template for your writeup if you want to submit it as a markdown file, but feel free to use some other method and submit a pdf if you prefer.

---


**The goals / steps of this project are the following:**  

**Training / Calibration**  

* Download the simulator and take data in "Training Mode"
* Test out the functions in the Jupyter Notebook provided
* Add functions to detect obstacles and samples of interest (golden rocks)
* Fill in the `process_image()` function with the appropriate image processing steps (perspective transform, color threshold etc.) to get from raw images to a map.  The `output_image` you create in this step should demonstrate that your mapping pipeline works.
* Use `moviepy` to process the images in your saved dataset with the `process_image()` function.  Include the video you produce as part of your submission.

**Autonomous Navigation / Mapping**

* Fill in the `perception_step()` function within the `perception.py` script with the appropriate image processing functions to create a map and update `Rover()` data (similar to what you did with `process_image()` in the notebook). 
* Fill in the `decision_step()` function within the `decision.py` script with conditional statements that take into consideration the outputs of the `perception_step()` in deciding how to issue throttle, brake and steering commands. 
* Iterate on your perception and decision function until your rover does a reasonable (need to define metric) job of navigating and mapping.  

[//]: # (Image References)

[image1]: ./misc/rover_image.jpg
[image2]: ./calibration_images/example_grid1.jpg
[image3]: ./calibration_images/example_rock1.jpg 
[fsm]: ./misc/D0E3C5A3-D4F0-4207-9E97-7DF045D5C0C4.jpeg

## [Rubric](https://review.udacity.com/#!/rubrics/916/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it!

### Notebook Analysis
#### 1. Run the functions provided in the notebook on test images (first with the test data provided, next on data you have recorded). Add/modify functions to allow for color selection of obstacles and rock samples.
Here is an example of how to include an image in your writeup.

![alt text][image1]

#### 1. Populate the `process_image()` function with the appropriate analysis steps to map pixels identifying navigable terrain, obstacles and rock samples into a worldmap.  Run `process_image()` on your test data using the `moviepy` functions provided to create video output of your result. 
And another! 

![alt text][image2]
### Autonomous Navigation and Mapping

#### 1. Fill in the `perception_step()` (at the bottom of the `perception.py` script) and `decision_step()` (in `decision.py`) functions in the autonomous mapping scripts and an explanation is provided in the writeup of how and why these functions were modified as they were.

`perception.py` was forked into two files, `perception.py` and `perception_step.py`, the latter computes additional rover properties using functions from the former.

Computed properties are as follows:

* `rover.mean_dir`: the name is completely wrong, it is the `k`th angle of visible terrain pixels. If `k` points to an angle larger than the mean angle (in the middle), the rover following that angle should prefer to "hug" the left wall. `k` is between 55% and 70% of the total number of visible pixels, determined by standard deviation of the angles. Narrower corridors get smaller `k`, so the rover drives more straight.

* `rover.home_dir`: rover-centric angle between current and starting position.

* `rover.home_mean_dir`: a combination of previous two, it's `home_dir` clipped to: the wall following angle of `mean_dir` and a right-wall-following angle computed with `k2 = 1 - k`. It's meant to drive the rover straight if home position is somewhere ahead, or close to one of the walls if it's not in the current view.

#### 2. Launching in autonomous mode your rover can navigate and map autonomously.  Explain your results and how you might improve them in your writeup.  
The following results were achieved on Linux with fantastic settings, 15-20 FPS and with one additional package installed (`pip install absl-py`, alternatively the flag code may be removed from drive_rover.py to run without it).

I've implemented the project with all extensions: picking up and returning the samples to base. I've been able to succesfully run it almost every time, with the exception of when the rocks are spawned next to each other, which might sometime confuse the rover. To solve this a "true" centroid algorithm could be implemented for processing the rock image, to filter out just one rock.

`decision.py` implements the following state machine:

![alt text][fsm]

The rover starts in `start` state, which will drive it forward until first obstacle is met. This is to prevent the situation where the rover would drive in circles, because it starts too far from any walls.

Next, the main loop of the program begins in state `forward`. The rover navigates according to `rover.mean_dir`, scaling the speed according to the number of terrain pixels available along the movement vector. The rover will transition to `brake` when the ratio of navigable pixels within short distance in front of it is too low. `brake` will stop the rover and turn until the ratio is raised again.

`forward` will transition to `brake_rock` whenever a `rover.rock_dir` is available and pointing to the left side of the rover or up to 30 degrees in the right-hand-side direction. This is to prevent the rover from losing it's original direction and skipping a part of the map.

`brake_rock` will stop the rover and wait until pitch and roll stabilise. If the rover was going too fast, it's possible that it will overshoot the rock. Therefore, if the rock is not visible after stopping the rover will attempt to back up. If the rock is visible, `turn_rock` will face it's direction and `forward_rock` will approach it. At any given time, if the rock is not visible for more than 0.5 second, the rover transitions back to `forward`.

In any state of the machine, if the rover is near rock, it will fully stop and pick up the rock. Only then normal operation resumes.

Whenever the rover picks up the last rock, machine will transition to `turn_home`. This state rotates the rover in the direction of starting position `rover.home_dir`. The next state is `forward_home` which acts as `forward`, except it will transition back to `turn_home` every 30s, use `rover.home_mean_dir` for navigation, and terminate when the distance to starting position is less or equal 5m.

All of the "movement" states (highlighted in red in the diagram) can additionally transition into the `unstuck` state. We determine the rover to be stuck, if the total distance traveled within last 3s is less than 0.5m (this is computed only when the rover is continuously in "movement" states). To unstuck, we stop, back up the rover and turn. Then the rover proceeds back to `forward` or `forward_home` state.

![alt text][image3]


