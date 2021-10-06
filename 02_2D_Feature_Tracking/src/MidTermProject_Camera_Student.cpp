/* INCLUDES FOR THIS PROJECT */
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>
#include <sstream>
#include <vector>

#include "dataStructures.h"
#include "matching2D.hpp"

using namespace std;

/* MAIN PROGRAM */
int main(int argc, const char *argv[]) {
  /* INIT VARIABLES AND DATA STRUCTURES */

  // data location
  string dataPath = "../";

  // camera
  string imgBasePath = dataPath + "images/";
  // left camera, color
  string imgPrefix = "KITTI/2011_09_26/image_00/data/000000";
  string imgFileType = ".png";
  // first file index to load (assumes Lidar and camera names have identical
  // naming convention)
  int imgStartIndex = 0;
  // last file index to load
  int imgEndIndex = 9;
  // no. of digits which make up the file index (e.g. img-0001.png)
  int imgFillWidth = 4;

  // misc
  // no. of images which are held in memory (ring buffer) at the same time
  int dataBufferSize = 2;
  // list of data frames which are held in memory at the same time
  vector<DataFrame> dataBuffer;
  // visualize results
  bool bVis = false;

  vector<std::string> detectorTypes = {"SHITOMASI", "HARRIS", "FAST", "BRISK",
                                       "ORB",       "AKAZE",  "SIFT"};
  vector<std::string> descriptorTypes = {"BRISK", "BRIEF", "ORB",
                                         "FREAK", "AKAZE", "SIFT"};
  vector<std::string> matcherTypes = {"MAT_BF"};
  vector<std::string> selectorTypes = {"SEL_KNN"};

  // set up the configurations through different combinition
  vector<Configuaration> configs;
  for (const auto &detector_type : detectorTypes) {
    for (const auto &descriptor_type : descriptorTypes) {
      for (const auto &matcher_type : matcherTypes) {
        for (const auto &selector_type : selectorTypes) {
          Configuaration config;
          config.detectorType = detector_type;
          config.descriptorType = descriptor_type;
          config.matcherType = matcher_type;
          config.selectorType = selector_type;
          configs.push_back(config);
        }
      }
    }
  }

  // prepare the file for output
  std::string outputFilename = "evaluation.csv";
  std::ofstream outputFile(outputFilename, ios::out);

  outputFile << "Detector Type"
             << ","
             << "Descriptor Type"
             << ","
             << "Frame ID"
             << ","
             << "Number of Keypoints"
             << ","
             << "Time for Keypoint Detection [ms]"
             << ","
             << "Time for Descriptor Extraction [ms]"
             << ","
             << "Number of Matched Points"
             << ","
             << "Time for Matching [ms]" << std::endl;

  for (Configuaration &config : configs) {
    std::string detectorType = config.detectorType;
    std::string descriptorType = config.descriptorType;
    std::string matcherType = config.matcherType;
    std::string selectorType = config.selectorType;

    cout << "//---- detector: " << detectorType
         << ", descriptor: " << descriptorType << ", matcher: " << matcherType
         << ", selector: " << selectorType << " ----//" << endl;

    if ((descriptorType.compare("AKAZE") == 0 &&
         detectorType.compare("AKAZE") != 0) ||
        (descriptorType.compare("ORB") == 0 &&
         detectorType.compare("SIFT") == 0))
      continue;

    /* MAIN LOOP OVER ALL IMAGES */
    /* LOAD IMAGE INTO BUFFER */
    dataBuffer.clear();
    for (size_t imgIndex = 0; imgIndex <= imgEndIndex - imgStartIndex;
         imgIndex++) {
      cout << "--- imgIndex: " << imgIndex << " ----" << endl;
      // assemble filenames for current index
      ostringstream imgNumber;
      imgNumber << setfill('0') << setw(imgFillWidth)
                << imgStartIndex + imgIndex;
      string imgFullFilename =
          imgBasePath + imgPrefix + imgNumber.str() + imgFileType;

      // load image from file and convert to grayscale
      cv::Mat img, imgGray;
      img = cv::imread(imgFullFilename);
      cv::cvtColor(img, imgGray, cv::COLOR_BGR2GRAY);

      /// STUDENT ASSIGNMENT
      /// TASK MP.1 -> replace the following code with ring buffer of size
      /// dataBufferSize

      // push image into data frame buffer
      DataFrame frame;
      frame.cameraImg = imgGray;

      if (dataBuffer.size() >= dataBufferSize) {
        dataBuffer.erase(dataBuffer.begin());
      }

      dataBuffer.push_back(frame);

      /// EOF STUDENT ASSIGNMENT
      cout << "#1 : LOAD IMAGE INTO BUFFER done" << endl;

      /* DETECT IMAGE KEYPOINTS */

      // extract 2D keypoints from current image create empty feature list for
      // current image
      vector<cv::KeyPoint> keypoints;

      /// STUDENT ASSIGNMENT
      /// TASK MP.2 -> add the following keypoint detectors in file
      /// matching2D.cpp and enable string-based selection based on detectorType
      /// -> HARRIS, FAST, BRISK, ORB, AKAZE, SIFT

      if (detectorType.compare("SHITOMASI") == 0) {
        detKeypointsShiTomasi(keypoints, imgGray, config, false);
      } else if (detectorType.compare("HARRIS") == 0) {
        detKeypointsHarris(keypoints, imgGray, config, false);
      } else if (detectorType.compare("FAST") == 0 ||
                 detectorType.compare("BRISK") == 0 ||
                 detectorType.compare("ORB") == 0 ||
                 detectorType.compare("AKAZE") == 0 ||
                 detectorType.compare("SIFT") == 0) {
        detKeypointsModern(keypoints, imgGray, detectorType, config, false);
      } else {
        throw invalid_argument("Given detector type is invalid - " +
                               detectorType);
      }
      /// EOF STUDENT ASSIGNMENT

      /// STUDENT ASSIGNMENT
      /// TASK MP.3 -> only keep keypoints on the preceding vehicle

      // only keep keypoints on the preceding vehicle
      bool bFocusOnVehicle = true;
      cv::Rect vehicleRect(535, 180, 180, 150);
      if (bFocusOnVehicle) {
        vector<cv::KeyPoint> bounded_keypoints;

        for (auto kpt : keypoints) {
          if (vehicleRect.contains(kpt.pt)) {
            bounded_keypoints.push_back(kpt);
          }
        }
        keypoints = bounded_keypoints;
      }

      /// EOF STUDENT ASSIGNMENT

      // optional : limit number of keypoints (helpful for debugging and
      // learning)
      bool bLimitKpts = false;
      if (bLimitKpts) {
        int maxKeypoints = 50;

        if (detectorType.compare("SHITOMASI") == 0) {
          // there is no response info, so keep the first 50 as they are
          // sorted in descending quality order
          keypoints.erase(keypoints.begin() + maxKeypoints, keypoints.end());
        }
        cv::KeyPointsFilter::retainBest(keypoints, maxKeypoints);
        cout << " NOTE: Keypoints have been limited!" << endl;
      }

      // push keypoints and descriptor for current frame to end of data buffer
      (dataBuffer.end() - 1)->keypoints = keypoints;
      cout << "#2 : DETECT KEYPOINTS done" << endl;

      /* EXTRACT KEYPOINT DESCRIPTORS */

      /// STUDENT ASSIGNMENT
      /// TASK MP.4 -> add the following descriptors in file matching2D.cpp and
      /// enable string-based selection based on descriptorType / -> BRIEF, ORB,
      /// FREAK, AKAZE, SIFT

      cv::Mat descriptors;
      // BRIEF, ORB, FREAK, AKAZE, SIFT
      descKeypoints((dataBuffer.end() - 1)->keypoints,
                    (dataBuffer.end() - 1)->cameraImg, descriptors,
                    descriptorType, config);
      /// EOF STUDENT ASSIGNMENT

      // push descriptors for current frame to end of data buffer
      (dataBuffer.end() - 1)->descriptors = descriptors;

      cout << "#3 : EXTRACT DESCRIPTORS done" << endl;

      // wait until at least two images have been processed
      if (dataBuffer.size() > 1) {
        /* MATCH KEYPOINT DESCRIPTORS */

        vector<cv::DMatch> matches;

        /// STUDENT ASSIGNMENT
        /// TASK MP.5 -> add FLANN matching in file matching2D.cpp
        /// TASK MP.6 -> add KNN match selection and perform descriptor distance
        /// ratio filtering with t=0.8 in file matching2D.cpp

        std::string descriptorStyle =
            (descriptorType.compare("SIFT") == 0) ? "DES_HOG" : "DES_BINARY";

        matchDescriptors((dataBuffer.end() - 2)->keypoints,
                         (dataBuffer.end() - 1)->keypoints,
                         (dataBuffer.end() - 2)->descriptors,
                         (dataBuffer.end() - 1)->descriptors, matches,
                         descriptorStyle, matcherType, selectorType, config);

        /// EOF STUDENT ASSIGNMENT

        // store matches in current data frame
        (dataBuffer.end() - 1)->kptMatches = matches;

        cout << "#4 : MATCH KEYPOINT DESCRIPTORS done" << endl;

        // visualize matches between current and previous image
        bVis = false;
        if (bVis) {
          cv::Mat matchImg = ((dataBuffer.end() - 1)->cameraImg).clone();
          cv::drawMatches((dataBuffer.end() - 2)->cameraImg,
                          (dataBuffer.end() - 2)->keypoints,
                          (dataBuffer.end() - 1)->cameraImg,
                          (dataBuffer.end() - 1)->keypoints, matches, matchImg,
                          cv::Scalar::all(-1), cv::Scalar::all(-1),
                          vector<char>(),
                          cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

          string windowName = "Matching keypoints between two camera images";
          cv::namedWindow(windowName, 7);
          cv::imshow(windowName, matchImg);
          cout << "Press key to continue to next image" << endl;
          // wait for key to be pressed
          cv::waitKey(0);
        }
        bVis = false;
      } else {
        config.numOfMatches.push_back(0);
        config.durationMatching.push_back(0.0);
      }
    }
    // eof loop over all images
  }

  for (const auto &config : configs) {
    // cout << config.detectorType << "," << config.descriptorType << ","
    //      << config.matcherType << "," << config.selectorType << ","
    //      << config.numOfKeypoints.size() << ","
    //      << config.durationDetection.size() << ","
    //      << config.durationDescriptorExtraction.size() << ","
    //      << config.numOfMatches.size() << "," <<
    //      config.durationMatching.size()
    //      << endl;
    for (int i = 0; i < 10; ++i) {
      if (config.numOfKeypoints.size() == 0)
        continue;

      outputFile << config.detectorType << "," << config.descriptorType << ","
                 << i << "," << config.numOfKeypoints[i] << "," << std::fixed
                 << std::setprecision(8) << config.durationDetection[i] << ","
                 << std::fixed << std::setprecision(8)
                 << config.durationDescriptorExtraction[i] << ","
                 << config.numOfMatches[i] << "," << std::fixed
                 << std::setprecision(8) << config.durationMatching[i] << endl;
    }
    outputFile << endl;
  }

  outputFile.close();

  return 0;
}
