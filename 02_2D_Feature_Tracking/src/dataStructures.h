#ifndef dataStructures_h
#define dataStructures_h

#include <opencv2/core.hpp>
#include <vector>

struct DataFrame { // represents the available sensor information at the same
                   // time instance
  // camera image
  cv::Mat cameraImg;
  // 2D keypoints within camera image
  std::vector<cv::KeyPoint> keypoints;
  // keypoint descriptors
  cv::Mat descriptors;
  // keypoint matches between previous and current frame
  std::vector<cv::DMatch> kptMatches;
};

struct Configuaration {
  std::string detectorType;
  std::string descriptorType;
  std::string matcherType;
  std::string selectorType;
  std::vector<int> numOfKeypoints;
  std::vector<int> numOfMatches;
  std::vector<double> durationDetection;
  std::vector<double> durationDescriptorExtraction;
  std::vector<double> durationMatching;
};

#endif /* dataStructures_h */
