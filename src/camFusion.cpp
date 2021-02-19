
#include <iostream>
#include <algorithm>
#include <numeric>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "camFusion.hpp"
#include "dataStructures.h"

using namespace std;


// Create groups of Lidar points whose projection into the camera falls into the same bounding box
void clusterLidarWithROI(std::vector<BoundingBox> &boundingBoxes, std::vector<LidarPoint> &lidarPoints, float shrinkFactor, cv::Mat &P_rect_xx, cv::Mat &R_rect_xx, cv::Mat &RT)
{
    // loop over all Lidar points and associate them to a 2D bounding box
    cv::Mat X(4, 1, cv::DataType<double>::type);
    cv::Mat Y(3, 1, cv::DataType<double>::type);
    bool debug = false; 
    for (auto it1 = lidarPoints.begin(); it1 != lidarPoints.end(); ++it1)
    {
        // assemble vector for matrix-vector-multiplication
        X.at<double>(0, 0) = it1->x;
        X.at<double>(1, 0) = it1->y;
        X.at<double>(2, 0) = it1->z;
        X.at<double>(3, 0) = 1;

        // project Lidar point into camera
        Y = P_rect_xx * R_rect_xx * RT * X;
        cv::Point pt;
        // pixel coordinates
        pt.x = Y.at<double>(0, 0) / Y.at<double>(2, 0); 
        pt.y = Y.at<double>(1, 0) / Y.at<double>(2, 0); 

        vector<vector<BoundingBox>::iterator> enclosingBoxes; // pointers to all bounding boxes which enclose the current Lidar point
        for (vector<BoundingBox>::iterator it2 = boundingBoxes.begin(); it2 != boundingBoxes.end(); ++it2)
        {
            // shrink current bounding box slightly to avoid having too many outlier points around the edges
            cv::Rect smallerBox;
            smallerBox.x = (*it2).roi.x + shrinkFactor * (*it2).roi.width / 2.0;
            smallerBox.y = (*it2).roi.y + shrinkFactor * (*it2).roi.height / 2.0;
            smallerBox.width = (*it2).roi.width * (1 - shrinkFactor);
            smallerBox.height = (*it2).roi.height * (1 - shrinkFactor);

            // check wether point is within current bounding box
            if (smallerBox.contains(pt))
            {
                enclosingBoxes.push_back(it2);
            }

        } // eof loop over all bounding boxes

        // check wether point has been enclosed by one or by multiple boxes
        if (enclosingBoxes.size() == 1)
        { 
            // add Lidar point to bounding box
            enclosingBoxes[0]->lidarPoints.push_back(*it1);
        }

    } // eof loop over all Lidar points
}

/* 
* The show3DObjects() function below can handle different output image sizes, but the text output has been manually tuned to fit the 2000x2000 size. It is adjustable.
* For instance, to use a 1000x1000 size, adjusting the text positions by dividing them by 2.
*/
void show3DObjects(std::vector<BoundingBox> &boundingBoxes, cv::Size worldSize, cv::Size imageSize, bool bWait)
{
    bool debug=false; 
    // create topview image
    cv::Mat topviewImg(imageSize, CV_8UC3, cv::Scalar(255, 255, 255));

    for(auto it1=boundingBoxes.begin(); it1!=boundingBoxes.end(); ++it1)
    {
        // create randomized color for current 3D object
        cv::RNG rng(it1->boxID);
        cv::Scalar currColor = cv::Scalar(rng.uniform(0,150), rng.uniform(0, 150), rng.uniform(0, 150));

        // plot Lidar points into top view image
        int top=1e8, left=1e8, bottom=0.0, right=0.0; 
        float xwmin=1e8, ywmin=1e8, ywmax=-1e8;
        for (auto it2 = it1->lidarPoints.begin(); it2 != it1->lidarPoints.end(); ++it2)
        {
            // world coordinates
            float xw = (*it2).x; // world position in m with x facing forward from sensor
            float yw = (*it2).y; // world position in m with y facing left from sensor
            xwmin = xwmin<xw ? xwmin : xw;
            ywmin = ywmin<yw ? ywmin : yw;
            ywmax = ywmax>yw ? ywmax : yw;

            // top-view coordinates
            int y = (-xw * imageSize.height / worldSize.height) + imageSize.height;
            int x = (-yw * imageSize.width / worldSize.width) + imageSize.width / 2;
               
            float zw = (*it2).z; // world position in m with y facing left from sensor

            // find enclosing rectangle
            top = top <y ? top : y;
            left = left <x ? left : x;
            bottom = bottom >y ? bottom : y;
            right = right >x ? right : x;

            // draw individual point
            cv::circle(topviewImg, cv::Point(x, y), 4, currColor, -1);
        }

        // draw enclosing rectangle
        cv::rectangle(topviewImg, cv::Point(left, top), cv::Point(right, bottom),cv::Scalar(0,0,0), 1);

        // augment object with some key data
        char str1[200], str2[200];
        sprintf(str1, "id=%d, #pts=%d", it1->boxID, (int)it1->lidarPoints.size());
        putText(topviewImg, str1, cv::Point2f(left, bottom + 50), cv::FONT_HERSHEY_SIMPLEX, 1, currColor);
        sprintf(str2, "xmin=%2.2f m, yw=%2.2f m", xwmin, ywmax-ywmin);
        putText(topviewImg, str2, cv::Point2f(left, bottom+100), cv::FONT_HERSHEY_SIMPLEX, 1, currColor); 
        cout<< "xmin=" <<xwmin <<", ywmax - ywmin="<< ywmax-ywmin<<endl;
    }

    // plot distance markers
    float lineSpacing = 2.0; // gap between distance markers
    int nMarkers = floor(worldSize.height / lineSpacing);
    for (size_t i = 0; i < nMarkers; ++i)
    {
        int y = (-(i * lineSpacing) * imageSize.height / worldSize.height) + imageSize.height;
        cv::line(topviewImg, cv::Point(0, y), cv::Point(imageSize.width, y), cv::Scalar(255, 0, 0));
    }

    // display image
    string windowName = "3D Objects";
    cv::namedWindow(windowName, 1);
    cv::imshow(windowName, topviewImg);
    if(debug) 
    {
      double t = (double)cv::getTickCount() ;
      string fileName = "../images/result/"+ to_string(t) +".png";
      cv::imwrite(fileName, topviewImg);
    }

    if(bWait)
    {
        cv::waitKey(0); // wait for key to be pressed
    }
}


// associate a given bounding box with the keypoints it contains
void clusterKptMatchesWithROI(BoundingBox &boundingBox, std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, std::vector<cv::DMatch> &kptMatches)
{
    bool debug=false; 
    double sum = 0;
   
    std::vector<double> distance;
 
  
    for (auto it = kptMatches.begin(); it != kptMatches.end(); ++it)
    {
        cv::KeyPoint kp = kptsCurr.at(it->trainIdx);
        
        if (boundingBox.roi.contains(cv::Point(kp.pt.x, kp.pt.y)))  //bounding boxes contains currFrame's keypoints
        {    
          boundingBox.kptMatches.push_back(*it);
        }
     }
  
    if(debug) cout << "Find " << kptMatches.size()  << " matches in the BoudingBoxes" << endl;
    ///calcualte euclidean distance between prev and  cur frame from matchedKeypoints
   
    for (auto it = boundingBox.kptMatches.begin(); it!=boundingBox.kptMatches.end(); ++it)
    {  
         // distance between matched keypoints
        cv::KeyPoint kp_Curr = kptsCurr.at(it->trainIdx);
        cv::KeyPoint kp_Prev = kptsPrev.at(it->queryIdx);
        double dist = cv::norm(kp_Curr.pt - kp_Prev.pt);
        sum += dist;

    }
  
    double mean = sum / boundingBox.kptMatches.size();
    if(debug) std::cout << "mean distance =" << mean << endl;
    double threshold = mean * 1.2;  
   
    for (auto it = boundingBox.kptMatches.begin(); it!=boundingBox.kptMatches.end(); )
    {
        cv::KeyPoint kp_Curr = kptsCurr.at(it->trainIdx);
        cv::KeyPoint kp_Prev = kptsPrev.at(it->queryIdx);
        double dist = cv::norm(kp_Curr.pt - kp_Prev.pt);
      
      if (dist>=threshold) 
      {    
         boundingBox.kptMatches.erase(it);
       }
      else {
            it++;
        }
    }
    if(debug) cout << "Retained Shorter distance:" << boundingBox.kptMatches.size()  << " matches" << endl;
}


// Compute time-to-collision (TTC) based on keypoint correspondences in successive images
void computeTTCCamera(std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, 
                      std::vector<cv::DMatch> kptMatches, double frameRate, double &TTC, cv::Mat *visImg)
{
    bool debug=false; 
    // compute distance ratios between all matched keypoints
    vector<double> distRatios; // stores the distance ratios for all keypoints between curr. and prev. frame
    for (auto it1 = kptMatches.begin(); it1 != kptMatches.end() - 1; ++it1)
    { // outer keypoint loop

        // get current keypoint and its matched partner in the prev. frame
        cv::KeyPoint kpOuterCurr = kptsCurr.at(it1->trainIdx);
        cv::KeyPoint kpOuterPrev = kptsPrev.at(it1->queryIdx);

        for (auto it2 = kptMatches.begin() + 1; it2 != kptMatches.end(); ++it2)
        { // inner keypoint loop

            double minDist = 100.0; // min. required distance

            // get next keypoint and its matched partner in the prev. frame
            cv::KeyPoint kpInnerCurr = kptsCurr.at(it2->trainIdx);
            cv::KeyPoint kpInnerPrev = kptsPrev.at(it2->queryIdx);

            // compute distances and distance ratios
            double distCurr = cv::norm(kpOuterCurr.pt - kpInnerCurr.pt);
            double distPrev = cv::norm(kpOuterPrev.pt - kpInnerPrev.pt);

            if (distPrev > std::numeric_limits<double>::epsilon() && distCurr >= minDist)
            { // avoid division by zero

                double distRatio = distCurr / distPrev;
                distRatios.push_back(distRatio);
            }
        } // eof inner loop over all matched kpts
    }     // eof outer loop over all matched kpts

    // only continue if list of distance ratios is not empty
    if (distRatios.size() == 0)
    {
        cout<<"No distRatios";
        TTC = NAN;
        return;
    }

    // compute camera-based TTC from distance ratios
      std::sort(distRatios.begin(), distRatios.end());
    long medIndex = floor(distRatios.size() / 2.0);
    double medDistRatio = distRatios.size() % 2 == 0 ? (distRatios[medIndex - 1] + distRatios[medIndex]) / 2.0 : distRatios[medIndex]; // compute median dist. ratio to remove outlier influence

    double dT = 1 / frameRate;
    
    if(medDistRatio==1)  
    { 
      TTC = NAN;
      if(debug) cout << " medDistRatio =1 " ; 
    }
    else  TTC = -dT / (1 - medDistRatio);
    if(debug) cout << "ttc-camera = " << TTC << "s" << endl;
}


void computeTTCLidar(std::vector<LidarPoint> &lidarPointsPrev,
                     std::vector<LidarPoint> &lidarPointsCurr, double frameRate, double &TTC)
{
    bool debug = false;
    // auxiliary variables
    double dT = 1/frameRate; // time between two measurements in seconds
    double laneWidth = 4.0;
    // find closest distance to Lidar points 
    double minXPrev = 1e9, minXCurr = 1e9;
    double avgXPrev = 0.0 , avgXCurr =0.0; //to remove outliars 
    int prev =0, curr =0; 
 
    // find closest distance to Lidar points within ego lane
    for(auto it=lidarPointsPrev.begin(); it!=lidarPointsPrev.end(); ++it) {
      if (abs(it->y) <= laneWidth / 2.0) {  //
        //minXPrev = minXPrev>it->x ? it->x : minXPrev;
        avgXPrev+=it->x;  //to remove outliars 
        prev=prev+1;
      }
    }
    // find closest distance to Lidar points within ego lane
    for(auto it=lidarPointsCurr.begin(); it!=lidarPointsCurr.end(); ++it) {
       if (abs(it->y) <= laneWidth / 2.0) {
        // minXCurr = minXCurr>it->x ? it->x : minXCurr;
         avgXCurr+=it->x;   //to remove outliars 
         curr=curr+1;
       }
    }
    if(prev!=0) minXPrev = avgXPrev/prev;
    if(curr!=0) minXCurr = avgXCurr/curr;
    // compute TTC from both measurements
    if(debug) cout << "minXCurr: " << minXCurr << endl;
    if(debug) cout << "minXPrev: " << minXPrev << endl;
    if(minXPrev-minXCurr ==0)
    { TTC = NAN;
     cout << "TTC from Lidar: NAN" << endl; 
    }
    else TTC = minXCurr *  dT/(minXPrev-minXCurr);
    if(debug) cout << "TTC from Lidar: " << TTC << endl;
  
}


void matchBoundingBoxes(std::vector<cv::DMatch> &matches, std::map<int, int> &bbBestMatches, DataFrame &prevFrame, DataFrame &currFrame)
{
    bool debug= false;
    int prv = prevFrame.boundingBoxes.size();  
    int cur = currFrame.boundingBoxes.size();  
    int pt_counts[prv][cur] = { };
    for (auto it = matches.begin(); it != matches.end() - 1; ++it)     {
        cv::KeyPoint query = prevFrame.keypoints[it->queryIdx];
        auto query_pt = cv::Point(query.pt.x, query.pt.y);
        bool query_found = false;
        cv::KeyPoint train = currFrame.keypoints[it->trainIdx];
        auto train_pt = cv::Point(train.pt.x, train.pt.y);
        bool train_found = false;
        std::vector<int> query_id, train_id;
        for (int i = 0; i < prv; i++) {
            if (prevFrame.boundingBoxes[i].roi.contains(query_pt))             {
                query_found = true;
                query_id.push_back(i);
             }
        }
        for (int i = 0; i < cur; i++) {
            if (currFrame.boundingBoxes[i].roi.contains(train_pt))             {
                train_found= true;
                train_id.push_back(i);
            }
        }
        if (query_found && train_found)
        {
            for (auto id_prev: query_id)
                for (auto id_curr: train_id)
                     pt_counts[id_prev][id_curr] += 1;
        }
    }

    for (int i = 0; i < prv; i++)
    {
         int max_count = 0;
         int id_max = 0;
         for (int j = 0; j < cur; j++)
             if (pt_counts[i][j] > max_count)  //highest number of keypoint correspondence are retained. 
             {
                  max_count = pt_counts[i][j];
                  id_max = j;
             }
          bbBestMatches[i] = id_max; //highest number of keypoint correspondence are stored in bbBestMatched. 
    }
    bool bMsg = false;
    if (bMsg)
        for (int i = 0; i < prv; i++)
             if(debug) cout << "Box " << i << " matches " << bbBestMatches[i]<< " points" << endl;
}
