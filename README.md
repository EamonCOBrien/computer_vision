# computer_vision

## How to run:

1) Launch gazebo: 
`roslaunch neato_gazebo neato_empty_world.launch load_camera:=true`

2) Add models from this repo's 'model' folder to the gazebo world.

3) Launch rqt window for visualizing from neato camera perspective:
`rosrun rqt_gui rqt_gui`

4) If you haven't before, compile the main script: 
`chmod u+x finite_state.py`

5) Run finite state controller: 
`rosrun computer_vision finite_state.py`

## Goal

We wanted to explore methods for recognizing and categorizing objects with computer vision, and thought it would be fun to make a Halloween themed project because we started in October. To combine the two, we programmed a trick-or-treating neato, which captures images of candy in its simulated environment and performs feature recognition and matching to categorize it based on training images.

## Object Recognition Methodology

Our original plan was to use a convolutional neural network to recognize different types of candy, but we pivoted towards feature detection algorithms after learning about SIFT in class. Both options are well-supported in Python and fairly easy to implement, but we knew less about feature detection and wanted to understand it better.

Instead of SIFT or its derivatives, we used the ORB (Oriented FAST and rotated BRIEF) algorithm, which operates about the same from the perspective of the user, but is not patented. ORB is also designed to be faster than standard SIFT, which is important for our application.

## Results

We incorporated our new code into the finite state controller that we wrote for the warmup project. In the previous version, the robot was in teleop mode until the user pressed a number key, which activated a variety of other behaviors. We added recognizing a candy and reacting to it as a new behavior, mapped to the '1' key. When it enters this state, the neato captures an image with its onboard camera, detects keypoints with ORB, and compares the top 5 keypoints with every candy in its training database. For each candy, the database has several images. We average the match quality of each example of a candy and choose the type with the minimum value.

![output-stats](/report_images/Output.png)

When the neato is initialized, it is given a list of likes and dislikes which include the candies in its database. If the candy is one of its likes, it moves toward the image. For example, the neato likes nerds. 

![neato-likes-nerds](https://github.com/EamonCOBrien/computer_vision/blob/main/report_images/nerds.gif)

If it dislikes the candy, the neato moves back and forth as though it were shaking its head no. The neato dislikes swedish fish.

![neato-dislikes-swedishfish](https://github.com/EamonCOBrien/computer_vision/blob/main/report_images/swedish%20fish.gif)

If the neato sees a candy that it's 'allergic' to, like Reeses, then it will run away. 

![neato-allergic-reeses](https://github.com/EamonCOBrien/computer_vision/blob/main/report_images/reeses(2).gif)

If the neato sees its favorite candy, Starburst, then it will spin in a circle out of excitedment.

![neato-fav-starburst](https://github.com/EamonCOBrien/computer_vision/blob/main/report_images/starburst.gif)

## Potential Improvements

Right now, the time it takes for the neato to check an image against its database is linear with the amount of training data it has, which can become very slow. It would be helpful to evaluate the accuracy of the algorithm's classifications against the number of images it has as training. Presumably, we could use this information to optimize the size of our training set.

Figure of runtime of image classification to the number of images per class: 
![figure](/report_images/runtime_vs_datasetsize.png)

A good way to speed up classificatin would be to cache the results of running ORB on our training data. At the moment, we are redoing all that work each time the function is called.
