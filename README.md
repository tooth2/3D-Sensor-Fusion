# 3D-Object-Tracking using Sensor Fusion(Camera+Lidar) 
Vehicle Tracking using YOLO v3 (image training and classification) and Lidar data


## Implementation Approach

### 1. Match 3D Object
* `matchBoundingBoxes` function 
    * takes as input both the previous and the current data frames 
    * provides as output the ids of the matched regions of interest (i.e. the boxID property). 
    * returns the specified output, where each bounding box is assigned the match candidate with the highest number of keypoint correspondences
    * after getting highest number of keypoint correspondence, these are stored in `bbBestMatched`.
    * For debug purpose, the number of bouding boxes is counted from `bbBestMatches.size()` 

### 2.Compute Lidar-based TTC
* `computeTTCLidar` function : Compute Lidar-based TTC
    * Compute the time-to-collision in second for all matched 3D objects using only Lidar measurements from the matched bounding boxes between current and previous frame 
    * from both previous and current fame, add closest distance to Lidar points
    * outlier Lidar points are excluded which are away more than +/- lanewidth/2 (i.e. left/right lanes). 
    * Even though Lidar is a reliable sensor, erroneous measurements may still occur. When searching for the closest points, such errorneous measurements will pose a problem as the estimated distance will be too small. Thurs from previous and current Frame, minXCurr and minXPrev are recorded to cope with a certain number of outliers. 
    * `TTC = minXCurr/(minXPrev-minXCurr) * 1/frameRate` from two successive frames, where frameRate is required to compute the delta time between frames and TTC will hold the result of the computation.
    * TTC computation from the Lidar sensor is based on two noisy distance measurements whereas Radar sensor is more accurate due to direct measurement from relative velocity.
    

### 3. Associate Keypoint Correspondences with Bounding Boxes
* `clusterKptMatchesWithROI` : associate a given bounding box with the keypoints it contains 
    * Prepare the TTC computation based on camera measurements by associating keypoint correspondences to the bounding boxes which enclose them. 
    * All matches which satisfy this condition are added to a vector ; by addding the keypoint correspondences to the "kptMatches" property of the respective bounding boxes
    * outlier matches are removed based on the computation of euclidean distance between previous and current frame from matched keypoints in the bounding box. 
    * compute a robust mean of all the euclidean distances between keypoint matches and then remove those that are too far away from the mean (`threshold = mean * 1.2`)
    * for debugging purpose, displayed the number of retained matched from `kptMatches.size()`
    
    
### 4. Compute Camera-based TTC   
* `computeTTCCamera`
    * Once keypoint matches have been added to the bounding boxes, the next step is to compute the TTC estimate.
    * Stored the distance ratios(`distRatio`) for all keypoints between current and previous frame
    * Computed the time-to-collision in second for all matched 3D objects using only keypoint correspondences from the matched bounding boxes between current and previous frame.
    * Computed median distance ratio to remove outlier influence to avoid severe estimation errors.
    * kptsPrev and kptsCurr are the input keypoint sets, kptMatches are the matches between the two sets.
    * `TTC = -1 / (1 - medDistRatio) * 1/frameRate` , where frameRate is required to compute the delta time between frames and TTC will hold the result of the computation

### Lidar Top View Result 
* `show3DObjects`:
    * handle output image sizes to fit the 500x1000 size. (can be adjustable to different sizes)
    * The result is as follows  : ![result](3Dobject.gif)
    
    
### 5. Performance Evaluation 1
1. Several examples(2-3) here the TTC estimate of the Lidar sensor does not seem plausible.
* several examples where the Lidar-based TTC estimate is way off.
* The assertion that the TTC is off is based on manually estimating the distance to the rear of the preceding vehicle from a top view perspective of the Lidar points.

### 6. Performance Evaluation 1
2. All detector / descriptor combinations combinations are compared with respect to the TTC estimate on a frame-by-frame basis.
* the best performing combinations 
* several examples where camera-based TTC estimation is way off.

## Result 
FAST detector + FREAK Descriptor + BF_Matcher

![result1](FAST-FREAK.gif)

SHITOMASI detector + FREAK Descriptor + BF_Matcher

![result2](SH-FREAK.gif)

## Runtime Environment Dependencies
* cmake >= 2.8
  * [how to install](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
* OpenCV >= 4.1
  * This must be compiled from source using the `-D OPENCV_ENABLE_NONFREE=ON` cmake flag for testing the SIFT and SURF detectors.
  * [OpenCV 4.1.0 source code](https://github.com/opencv/opencv/tree/4.1.0)
* gcc/g++ >= 5.4
* yolov3.weights : Download `wget https://pjreddie.com/media/files/yolov3.weights `

### Rerefence 
* [Feature matching: Brute Force Matcher](https://docs.opencv.org/3.4/dc/dc3/tutorial_py_matcher.html)
* [Detector and Descriptor](https://docs.opencv.org/2.4/modules/features2d/doc/feature_detection_and_description.html)
* [Detector and matching](https://medium.com/data-breach/introduction-to-feature-detection-and-matching-65e27179885d)
* [YOLO website](https://pjreddie.com/darknet/yolo/)
* [YOLOv3: An Incremental Improvement](https://arxiv.org/abs/1804.02767)



