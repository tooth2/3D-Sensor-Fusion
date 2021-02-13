# 3D-Object-Tracking using Sensor Fusion(Camera+Lidar) 
Vehicle Tracking using YOLO v3 (image training and classification) and Lidar data


## Implementation Approach

### Match 3D Object
* `matchBoundingBoxes`
    * takes as input both the previous and the current data frames 
    * provides as output the ids of the matched regions of interest (i.e. the boxID property). 
    * returns the specified output, where each bounding box is assigned the match candidate with the highest number of keypoint correspondences
    
* `computeTTCLidar` : Compute Lidar-based TTC
    * Compute the time-to-collision in second for all matched 3D objects using only Lidar measurements from the matched bounding boxes between current and previous frame
    * outlier Lidar points are removed in a statistically robust way to avoid severe estimation errors.
    * outliers might be way too close and thus lead to faulty estimates of the TTC.
    
* `computeTTCCamera`
    * Once keypoint matches have been added to the bounding boxes, the next step is to compute the TTC estimate.
    * Compute the time-to-collision in second for all matched 3D objects using only keypoint correspondences from the matched bounding boxes between current and previous frame.
    * outlier correspondences are removed in a statistically robust way to avoid severe estimation errors.

* `clusterKptMatchesWithROI` : associate a given bounding box with the keypoints it contains 
    * Prepare the TTC computation based on camera measurements by associating keypoint correspondences to the bounding boxes which enclose them. 
    * All matches which satisfy this condition are added to a vector ; by addding the keypoint correspondences to the "kptMatches" property of the respective bounding boxes
    * outlier matches are removed based on the euclidean distance between them in relation to all the matches in the bounding box.
    * compute a robust mean of all the euclidean distances between keypoint matches and then remove those that are too far away from the mean.
    
* `show3DObjects`:
    * handle output image sizes to fit the 2000x2000 size. (can be adjustable to different sizes)
    
### Performance Evaluation 
1. Several examples(2-3) here the TTC estimate of the Lidar sensor does not seem plausible.
* several examples where the Lidar-based TTC estimate is way off.
* The assertion that the TTC is off is based on manually estimating the distance to the rear of the preceding vehicle from a top view perspective of the Lidar points.

2. All detector / descriptor combinations combinations are compared with respect to the TTC estimate on a frame-by-frame basis.
* the best performing combinations 
* several examples where camera-based TTC estimation is way off.

## Result 
One example , FAST detector + SIFT Descriptor + BF_Matcher

![result](result.gif)

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



