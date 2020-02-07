 # Monocular-Visual-Odometry

# The goal of the project
This project tries to use monocular visual odometry to track the trajectory of robot and map the trajectory in a 2-D image.

The review article extract data from KITTI dataset, and our group try to implement it in real time by using Kinect in robot.
Because we use monocular visual, we can just get the relative positions of the points and it is not possible to obtain the absolute scale of the trajectory (Nister, 2004). This project can be used in the situation which GPS and wheel odometry become unreliable and it also can be used to calibrate the position with other methods.

# Review paper
The review articles are Monocular Visual Odometry using OpenCV (Singh, 2015) and An Efficient Solution to the Five-Point Relative Pose Problem (Nister, 2004). This project is using monocular visual odometry to track the robot motion trajectory in a 2-D image. In Singh’s article, the input dataset is from KITTI. The dataset has been undistorted. The algorithm can be concluded into six steps.

Firstly, capture the images 𝐼 and 𝐼.

Secondly, undistort the images. Because the images from KITTI have been undistorted, this step can be ignored. However, when we do track in real time, this step is necessary.

Thirdly, use FAST algorithm to detect features in image 𝑰 , then use KLT tracker to track these features in the next image 𝑰 . It the numbers of features in image 𝑰 is smaller than a threshold,a new detection will be triggered.

Fourthly, calculate the essential matrix by using five-point algorithm (Nister, 2004). Before it, use RANSAC (Random sample consensus) to fit the corresponding feature points between two images.

Fifthly, use SVD (singular value decomposition) to estimate R, t from the essential matrix.

Finally, get scale information from some external source (like a speedometer.

