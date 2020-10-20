## Welcome!

This GitHub repository contains the code used by [FTC team 8668 Should Be Fine](https://www.error404robotics.com) for the FIRST Tech Challenge SKYSTONE (2019-2020) competition season. This repository contains the public FTC SDK for the SKYSTONE (2019-2020) competition season.

Formerly this repository was hosted on Bitbucket, but was migrated to GitHub after the conclusion of the 2019-2020 season.

## Key Algorithms
### Pure Pursuit Autonomous Algorithm
The SBF Pure Pursuit algorithm is a vector-based navigational system first developed by Carnegie Mellon University, and subsequently adapted by us.

 Implements a pursuit robot maneuvering algorithm based on the steering behavior algorithm originally published by Craig Reynolds in ["Steering Behavior for Autonomous Characters"](http://www.red3d.com/cwr/papers/1999/gdc99steer.pdf).

Pure pursuit uses a fusion of positional information (x and y values, via two encoders) and heading (provided by the IMU). The algorithm is given a series of coordinate points along with a desired heading for each point. From the points, a path is constructed. The robot then uses the current robot position and the desired robot position (the next point on the path) to construct a movement vector. This means that if the robot is knocked off course, as long as the encoders track the movement, the robot can recover and return to the desired path.
#### [Pursuit.java](https://github.com/iamlordvoldemort/SkyStone_8668/blob/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Pursuit.java)
#### [Path.java](https://github.com/iamlordvoldemort/SkyStone_8668/blob/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Path.java)

### Generic Action Engine with Zero-Compile Autonomous Programming
For our autonomous code, we built an Action Engine that does not use hard-coded states and transitions encoded in our Java classes.  Instead, the Action Engine loads a CSV file from the phone’s internal storage to define the actions and transitions that the robot executes to perform the desired tasks.  Instead of dealing with tangled Java syntax for 80+ states and transitions, we are able to neatly organize them in a spreadsheet program.  Then we export the spreadsheet as a CSV file and load this file onto the phone.
During Init, our code takes whichever CSV file is selected and reads through it and builds a map of all the robot actions defined in the CSV file.  The Action Engine then loads each robot action into a run list and executes them sequentially, following the information provided in each action’s parameters to know which action to load and execute next. After an action is executed, the Action Engine removes it from the run list and the next listed action is loaded in its place.  Adapting our autonomous is as easy as changing a few rows in a CSV file using Excel.
#### [ActionMaster.java](https://github.com/iamlordvoldemort/SkyStone_8668/blob/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/sbfActions/ActionMaster.java)
#### [RobotAction.java](https://github.com/iamlordvoldemort/SkyStone_8668/blob/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/sbfActions/RobotAction.java)
#### [SBFActions](https://github.com/iamlordvoldemort/SkyStone_8668/tree/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/sbfActions)

### OpenCV Image Process for SkyStone Detection
We use OpenCV image processing where the webcam image is put through several filters that makes the 3 stones in the image turn black or white based on the color and brightness of each stone. As a result of the filtering process, the SkyStone shows up as black while the normal stones appear white in the processed image.  In order to increase robustness and to automatically account for varying lighting conditions, the numeric pixel values for each stone’s pixel region on the screen are added together resulting in a single value for each stone.  The three summed values are compared, and the lowest value represents the darkest stone.  Since the SkyStone is covered with dark decal, the darkest stone is chosen as the SkyStone.  The image processing activity occurs continuously during Init, so the robot already knows the SkyStone position when the autonomous period begins.
#### [CameraVision.java](https://github.com/iamlordvoldemort/SkyStone_8668/blob/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/sbfHardware/CameraVision.java)
