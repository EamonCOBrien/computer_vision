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

We created our own dataset by taking images of candy we had at home, from different angles, backgrounds, and lighting. Each candy type has at least 10 images in its dataset, which would be too small for a neural network, but has worked decently with keypoint matching.

## Results

Since our goal was to create a trick-or-treating robot, we wanted to program the neato to mimic the behavior of a child with specific candy preferences, as it visits different locations for receiving candy.

In order to create this behavior, we wanted the user to be able to navigate the neato around using teleop, and watch the neato in rqt, similar to a first-person video game. In order to alternate between teleop and other behaviors, we incorporated our image classification code into the finite state controller that we wrote for the warmup project. In the previous version, the robot was in teleop mode until the user pressed a number key, which activated a variety of other behaviors. We added recognizing a candy type and reacting to it as a new behavior, mapped to the '1' key. When it enters this state, the neato captures an image with its onboard camera, detects keypoint descriptors with ORB, and compares the top 5 keypoints with every candy in its training database. In `candy_classifier.py`, changing the 'dataset_size' variable will change the number of images used from each class in the training dataset. We average the match quality metric of each possible candy type and choose the type with the overall minimum value in order to get the best match.

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

Right now, the time it takes for the neato to check an image against its database is linear with the amount of training data it has, which can become very slow. We evaluated the accuracy of the algorithm's classifications against the number of images it has as training, and discovered that 5 images/class is when it reaches its peak performance. The graph below illustrates the linear time cost of adding more images to the training set.

Figure of runtime of image classification to the number of images per class: 
![figure](/report_images/runtime_vs_datasetsize.png)

A good way to speed up classification would be to cache the results of running ORB on our training data, beacuse at the moment, we are redoing all that work each time the function is called. Another way we have thought about as a way to speed up classification would be to create an eigen-image for each of the candy types, and perform keypoint matching after transforming the query image into eigenspace. This way, the dataset would precompile an eigen-image for each candy class at initialization, and then perform keypoint matching once per class for each query image (O(n)), as opposed to the current method, which is essentially O(n^2). 
