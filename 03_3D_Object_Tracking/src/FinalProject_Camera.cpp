
/* INCLUDES FOR THIS PROJECT */
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <numeric>
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>
#include <sstream>
#include <sys/stat.h>
#include <vector>

#include "camFusion.hpp"
#include "dataStructures.h"
#include "lidarData.hpp"
#include "matching2D.hpp"
#include "objectDetection2D.hpp"

using namespace std;

/* MAIN PROGRAM */
int main(int argc, const char *argv[]) {
  /* INIT VARIABLES AND DATA STRUCTURES */

  // data location
  string dataPath = "../";

  // camera
  string imgBasePath = dataPath + "images/";
  // left camera, color
  string imgPrefix = "KITTI/2011_09_26/image_02/data/000000";
  string imgFileType = ".png";
  // first file index to load (assumes Lidar and camera names have identical
  // naming convention)
  int imgStartIndex = 0;
  // last file index to load
  int imgEndIndex = 18;
  int imgStepWidth = 1;
  // no. of digits which make up the file index (e.g. img-0001.png)
  int imgFillWidth = 4;

  // object detection
  string yoloBasePath = dataPath + "dat/yolo/";
  string yoloClassesFile = yoloBasePath + "coco.names";
  string yoloModelConfiguration = yoloBasePath + "yolov3.cfg";
  string yoloModelWeights = yoloBasePath + "yolov3.weights";

  // Lidar
  string lidarPrefix = "KITTI/2011_09_26/velodyne_points/data/000000";
  string lidarFileType = ".bin";

  // calibration data for camera and lidar
  // 3x4 projection matrix after rectification
  cv::Mat P_rect_00(3, 4, cv::DataType<double>::type);
  // 3x3 rectifying rotation to make image planes co-planar
  cv::Mat R_rect_00(4, 4, cv::DataType<double>::type);
  // rotation matrix and translation vector
  cv::Mat RT(4, 4, cv::DataType<double>::type);

  RT.at<double>(0, 0) = 7.533745e-03;
  RT.at<double>(0, 1) = -9.999714e-01;
  RT.at<double>(0, 2) = -6.166020e-04;
  RT.at<double>(0, 3) = -4.069766e-03;
  RT.at<double>(1, 0) = 1.480249e-02;
  RT.at<double>(1, 1) = 7.280733e-04;
  RT.at<double>(1, 2) = -9.998902e-01;
  RT.at<double>(1, 3) = -7.631618e-02;
  RT.at<double>(2, 0) = 9.998621e-01;
  RT.at<double>(2, 1) = 7.523790e-03;
  RT.at<double>(2, 2) = 1.480755e-02;
  RT.at<double>(2, 3) = -2.717806e-01;
  RT.at<double>(3, 0) = 0.0;
  RT.at<double>(3, 1) = 0.0;
  RT.at<double>(3, 2) = 0.0;
  RT.at<double>(3, 3) = 1.0;

  R_rect_00.at<double>(0, 0) = 9.999239e-01;
  R_rect_00.at<double>(0, 1) = 9.837760e-03;
  R_rect_00.at<double>(0, 2) = -7.445048e-03;
  R_rect_00.at<double>(0, 3) = 0.0;
  R_rect_00.at<double>(1, 0) = -9.869795e-03;
  R_rect_00.at<double>(1, 1) = 9.999421e-01;
  R_rect_00.at<double>(1, 2) = -4.278459e-03;
  R_rect_00.at<double>(1, 3) = 0.0;
  R_rect_00.at<double>(2, 0) = 7.402527e-03;
  R_rect_00.at<double>(2, 1) = 4.351614e-03;
  R_rect_00.at<double>(2, 2) = 9.999631e-01;
  R_rect_00.at<double>(2, 3) = 0.0;
  R_rect_00.at<double>(3, 0) = 0;
  R_rect_00.at<double>(3, 1) = 0;
  R_rect_00.at<double>(3, 2) = 0;
  R_rect_00.at<double>(3, 3) = 1;

  P_rect_00.at<double>(0, 0) = 7.215377e+02;
  P_rect_00.at<double>(0, 1) = 0.000000e+00;
  P_rect_00.at<double>(0, 2) = 6.095593e+02;
  P_rect_00.at<double>(0, 3) = 0.000000e+00;
  P_rect_00.at<double>(1, 0) = 0.000000e+00;
  P_rect_00.at<double>(1, 1) = 7.215377e+02;
  P_rect_00.at<double>(1, 2) = 1.728540e+02;
  P_rect_00.at<double>(1, 3) = 0.000000e+00;
  P_rect_00.at<double>(2, 0) = 0.000000e+00;
  P_rect_00.at<double>(2, 1) = 0.000000e+00;
  P_rect_00.at<double>(2, 2) = 1.000000e+00;
  P_rect_00.at<double>(2, 3) = 0.000000e+00;

  // misc
  // frames per second for Lidar and camera
  double sensorFrameRate = 10.0 / imgStepWidth;
  // no. of images which are held in memory (ring buffer) at the same time
  int dataBufferSize = 2;
  // list of data frames which are held in memory at the same time
  vector<DataFrame> dataBuffer;
  // visualize results
  bool bVis = false;
  // save image output locally
  bool bSave = true;
  string imageOutputDir;

  vector<std::string> detectorTypes = {"SHITOMASI", "HARRIS", "FAST", "BRISK",
                                       "ORB",       "AKAZE",  "SIFT"};
  vector<std::string> descriptorTypes = {"BRISK", "BRIEF", "ORB",
                                         "FREAK", "AKAZE", "SIFT"};
  vector<std::string> matcherTypes = {"MAT_BF"};
  vector<std::string> selectorTypes = {"SEL_KNN"};

  // for testing
  // vector<std::string> detectorTypes = {"ORB"};
  // vector<std::string> descriptorTypes = {"SIFT"};
  // vector<std::string> matcherTypes = {"MAT_BF"};
  // vector<std::string> selectorTypes = {"SEL_KNN"};

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
  std::string summaryFilename = "../output/summary.csv";
  std::string comparisonFilename = "../output/comparison.csv";
  std::ofstream summaryFile(summaryFilename, ios::out);
  std::ofstream comparisonFile(comparisonFilename, ios::out);

  summaryFile << "Detector Type"
              << ","
              << "Descriptor Type"
              << ","
              << "Frame ID"
              << ","
              << "Lidar-Based TTC [ms]"
              << ","
              << "Camera-Based TTC [ms]" << endl;

  comparisonFile << "Detector Type"
                 << ","
                 << "Descriptor Type"
                 << ","
                 << "Minimal Lidar-Based TTC [ms]"
                 << ","
                 << "Minimal Camera-Based TTC [ms]"
                 << ","
                 << "Maximal Lidar-Based TTC [ms]"
                 << ","
                 << "Maximal Camera-Based TTC [ms]"
                 << ","
                 << "Mean Lidar-Based TTC [ms]"
                 << ","
                 << "Mean Camera-Based TTC [ms]"
                 << ","
                 << "Median Lidar-Based TTC [ms]"
                 << ","
                 << "Median Camera-Based TTC [ms]"
                 << ","
                 << "Standard Deviation Lidar-Based TTC [ms]"
                 << ","
                 << "Standard Deviation Camera-Based TTC [ms]" << endl;

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

    if (bSave) {
      struct stat st = {0};
      imageOutputDir = dataPath + "output/";
      const char *outputDir = imageOutputDir.c_str();
      if (stat(outputDir, &st) == -1) {
        const int dir_err =
            mkdir(outputDir, S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
        if (-1 == dir_err) {
          cout << "Error creating directory " << imageOutputDir << endl;
          exit(1);
        }
      }

      imageOutputDir +=
          "detector_" + detectorType + "_descriptor_" + descriptorType;
      outputDir = imageOutputDir.c_str();
      if (stat(outputDir, &st) == -1) {
        const int dir_err =
            mkdir(outputDir, S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
        if (-1 == dir_err) {
          cout << "Error creating directory " << imageOutputDir << endl;
          exit(1);
        }
      }
    }

    /* MAIN LOOP OVER ALL IMAGES */

    dataBuffer.clear();
    for (size_t imgIndex = 0; imgIndex <= imgEndIndex - imgStartIndex;
         imgIndex += imgStepWidth) {
      /* LOAD IMAGE INTO BUFFER */

      cout << "--- imgIndex: " << imgIndex << " ----" << endl;

      // assemble filenames for current index
      ostringstream imgNumber;
      imgNumber << setfill('0') << setw(imgFillWidth)
                << imgStartIndex + imgIndex;
      string imgFullFilename =
          imgBasePath + imgPrefix + imgNumber.str() + imgFileType;

      // load image from file
      cv::Mat img = cv::imread(imgFullFilename);

      // push image into data frame buffer
      DataFrame frame;
      frame.cameraImg = img;

      dataBuffer.push_back(frame);

      cout << "#1 : LOAD IMAGE INTO BUFFER done" << endl;

      /* DETECT & CLASSIFY OBJECTS */

      float confThreshold = 0.2;
      float nmsThreshold = 0.4;
      detectObjects((dataBuffer.end() - 1)->cameraImg,
                    (dataBuffer.end() - 1)->boundingBoxes, confThreshold,
                    nmsThreshold, yoloBasePath, yoloClassesFile,
                    yoloModelConfiguration, yoloModelWeights, bVis);

      cout << "#2 : DETECT & CLASSIFY OBJECTS done" << endl;

      /* CROP LIDAR POINTS */

      // load 3D Lidar points from file
      string lidarFullFilename =
          imgBasePath + lidarPrefix + imgNumber.str() + lidarFileType;
      std::vector<LidarPoint> lidarPoints;
      loadLidarFromFile(lidarPoints, lidarFullFilename);

      // remove Lidar points based on distance properties
      // focus on ego lane
      float minZ = -1.5, maxZ = -0.9, minX = 2.0, maxX = 20.0, maxY = 2.0,
            minR = 0.1;
      cropLidarPoints(lidarPoints, minX, maxX, maxY, minZ, maxZ, minR);

      (dataBuffer.end() - 1)->lidarPoints = lidarPoints;

      cout << "#3 : CROP LIDAR POINTS done" << endl;

      /* CLUSTER LIDAR POINT CLOUD */

      // associate Lidar points with camera-based ROI
      // shrinks each bounding box by the given percentage to avoid 3D object
      // merging at the edges of an ROI
      float shrinkFactor = 0.10;
      clusterLidarWithROI((dataBuffer.end() - 1)->boundingBoxes,
                          (dataBuffer.end() - 1)->lidarPoints, shrinkFactor,
                          P_rect_00, R_rect_00, RT);

      // Visualize 3D objects
      bVis = false;
      if (bVis) {
        show3DObjects((dataBuffer.end() - 1)->boundingBoxes,
                      cv::Size(4.0, 20.0), cv::Size(2000, 2000), false);
      }
      bVis = false;

      cout << "#4 : CLUSTER LIDAR POINT CLOUD done" << endl;

      // REMOVE THIS LINE BEFORE PROCEEDING WITH THE FINAL PROJECT
      // skips directly to the next image without processing what comes beneath
      // continue;

      /* DETECT IMAGE KEYPOINTS */

      // convert current image to grayscale
      cv::Mat imgGray;
      cv::cvtColor((dataBuffer.end() - 1)->cameraImg, imgGray,
                   cv::COLOR_BGR2GRAY);

      // extract 2D keypoints from current image
      // create empty feature list for current image
      vector<cv::KeyPoint> keypoints;

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

      cout << "#5 : DETECT KEYPOINTS done" << endl;

      /* EXTRACT KEYPOINT DESCRIPTORS */

      cv::Mat descriptors;
      descKeypoints((dataBuffer.end() - 1)->keypoints,
                    (dataBuffer.end() - 1)->cameraImg, descriptors,
                    descriptorType, config);

      // push descriptors for current frame to end of data buffer
      (dataBuffer.end() - 1)->descriptors = descriptors;

      cout << "#6 : EXTRACT DESCRIPTORS done" << endl;

      // wait until at least two images have been processed
      if (dataBuffer.size() > 1) {
        /* MATCH KEYPOINT DESCRIPTORS */

        vector<cv::DMatch> matches;
        std::string descriptorStyle =
            descriptorType.compare("SIFT") == 0 ? "DES_HOG" : "DES_BINARY";

        matchDescriptors((dataBuffer.end() - 2)->keypoints,
                         (dataBuffer.end() - 1)->keypoints,
                         (dataBuffer.end() - 2)->descriptors,
                         (dataBuffer.end() - 1)->descriptors, matches,
                         descriptorStyle, matcherType, selectorType, config);

        // store matches in current data frame
        (dataBuffer.end() - 1)->kptMatches = matches;

        cout << "#7 : MATCH KEYPOINT DESCRIPTORS done" << endl;

        /* TRACK 3D OBJECT BOUNDING BOXES */

        //// STUDENT ASSIGNMENT
        //// TASK FP.1 -> match list of 3D objects (vector<BoundingBox>) between
        /// current and previous frame (implement ->matchBoundingBoxes)
        map<int, int> bbBestMatches;
        // associate bounding boxes between current and previous frame using
        // keypoint matches
        matchBoundingBoxes(matches, bbBestMatches, *(dataBuffer.end() - 2),
                           *(dataBuffer.end() - 1), config);
        //// EOF STUDENT ASSIGNMENT

        // store matches in current data frame
        (dataBuffer.end() - 1)->bbMatches = bbBestMatches;

        cout << "#8 : TRACK 3D OBJECT BOUNDING BOXES done" << endl;

        /* COMPUTE TTC ON OBJECT IN FRONT */

        // loop over all BB match pairs
        for (auto it1 = (dataBuffer.end() - 1)->bbMatches.begin();
             it1 != (dataBuffer.end() - 1)->bbMatches.end(); ++it1) {
          // find bounding boxes associates with current match
          BoundingBox *prevBB, *currBB;
          for (auto it2 = (dataBuffer.end() - 1)->boundingBoxes.begin();
               it2 != (dataBuffer.end() - 1)->boundingBoxes.end(); ++it2) {
            // check wether current match partner corresponds to this BB
            if (it1->second == it2->boxID) {
              currBB = &(*it2);
            }
          }

          for (auto it2 = (dataBuffer.end() - 2)->boundingBoxes.begin();
               it2 != (dataBuffer.end() - 2)->boundingBoxes.end(); ++it2) {
            // check wether current match partner corresponds to this BB
            if (it1->first == it2->boxID) {
              prevBB = &(*it2);
            }
          }

          // compute TTC for current match
          // only compute TTC if we have Lidar points
          if (currBB->lidarPoints.size() > 0 &&
              prevBB->lidarPoints.size() > 0) {
            //// STUDENT ASSIGNMENT
            //// TASK FP.2 -> compute time-to-collision based on Lidar data
            //// (implement -> computeTTCLidar)
            double ttcLidar;
            computeTTCLidar(prevBB->lidarPoints, currBB->lidarPoints,
                            sensorFrameRate, ttcLidar);
            config.LidarTTC.push_back(ttcLidar);
            //// EOF STUDENT ASSIGNMENT

            //// STUDENT ASSIGNMENT
            //// TASK FP.3 -> assign enclosed keypoint matches to bounding box
            //// (implement -> clusterKptMatchesWithROI)
            //// TASK FP.4 -> compute time-to-collision based on camera
            //// (implement -> computeTTCCamera)
            double ttcCamera;
            clusterKptMatchesWithROI(*currBB, (dataBuffer.end() - 2)->keypoints,
                                     (dataBuffer.end() - 1)->keypoints,
                                     (dataBuffer.end() - 1)->kptMatches);
            computeTTCCamera((dataBuffer.end() - 2)->keypoints,
                             (dataBuffer.end() - 1)->keypoints,
                             currBB->kptMatches, sensorFrameRate, ttcCamera);
            config.cameraTTC.push_back(ttcCamera);
            //// EOF STUDENT ASSIGNMENT

            bVis = false;
            if (bVis) {
              cv::Mat visImg = (dataBuffer.end() - 1)->cameraImg.clone();
              showLidarImgOverlay(visImg, currBB->lidarPoints, P_rect_00,
                                  R_rect_00, RT, &visImg);
              cv::rectangle(visImg, cv::Point(currBB->roi.x, currBB->roi.y),
                            cv::Point(currBB->roi.x + currBB->roi.width,
                                      currBB->roi.y + currBB->roi.height),
                            cv::Scalar(0, 255, 0), 2);

              string windowName = "Final Results : TTC";
              cv::namedWindow(windowName, 4);
              cv::imshow(windowName, visImg);
              cout << "Press key to continue to next frame" << endl;
              cv::waitKey(0);
            }
            bVis = false;

            if (bSave) {
              cv::Mat saveImg = (dataBuffer.end() - 1)->cameraImg.clone();
              showLidarImgOverlay(saveImg, currBB->lidarPoints, P_rect_00,
                                  R_rect_00, RT, &saveImg);
              cv::rectangle(saveImg, cv::Point(currBB->roi.x, currBB->roi.y),
                            cv::Point(currBB->roi.x + currBB->roi.width,
                                      currBB->roi.y + currBB->roi.height),
                            cv::Scalar(0, 255, 0), 2);

              string imageName =
                  imageOutputDir + "/000000" + imgNumber.str() + imgFileType;
              const char *imgFilename = imageName.c_str();
              cv::imwrite(imgFilename, saveImg);
            }
          } // eof TTC computation

        } // eof loop over all BB matches
      } else {
        config.numOfMatches.push_back(0);
        config.durationDescriptorMatch.push_back(0.0);
        config.durationMatchBoundingBoxes.push_back(0.0);
        config.LidarTTC.push_back(0.0);
        config.cameraTTC.push_back(0.0);
      }

    } // eof loop over all images
  }   // eof loop over all configs

  for (const auto &config : configs) {
    if (config.numOfKeypoints.size() == 0)
      continue;

    vector<double> copiedLidarTTC;
    vector<double> copiedCameraTTC;

    for (int i = 1; i < imgEndIndex; ++i) {
      if (!isnan(config.LidarTTC[i]))
        copiedLidarTTC.push_back(config.LidarTTC[i]);
      if (!isnan(config.cameraTTC[i]))
        copiedCameraTTC.push_back(config.cameraTTC[i]);
    }

    double maxTTCLidar =
        *max_element(copiedLidarTTC.begin(), copiedLidarTTC.end());
    double maxTTCCamera =
        *max_element(copiedCameraTTC.begin(), copiedCameraTTC.end());

    double minTTCLidar =
        *min_element(copiedLidarTTC.begin(), copiedLidarTTC.end());
    double minTTCCamera =
        *min_element(copiedCameraTTC.begin(), copiedCameraTTC.end());

    double meanTTCLidar =
        accumulate(copiedLidarTTC.begin(), copiedLidarTTC.end(), 0.0) /
        copiedLidarTTC.size();
    double meanTTCCamera =
        accumulate(copiedCameraTTC.begin(), copiedCameraTTC.end(), 0.0) /
        copiedCameraTTC.size();

    double stdevTTCLidar = 0, stdevTTCCamera = 0;

    for (int i = 0; i < copiedLidarTTC.size(); ++i) {
      stdevTTCLidar += (copiedLidarTTC[i] - meanTTCLidar) *
                       (copiedLidarTTC[i] - meanTTCLidar);
    }
    for (int i = 0; i < copiedCameraTTC.size(); ++i) {
      stdevTTCCamera += (copiedCameraTTC[i] - meanTTCCamera) *
                        (copiedCameraTTC[i] - meanTTCCamera);
    }

    stdevTTCLidar /= copiedLidarTTC.size();
    stdevTTCCamera /= copiedCameraTTC.size();

    sort(copiedLidarTTC.begin(), copiedLidarTTC.end());
    sort(copiedCameraTTC.begin(), copiedCameraTTC.end());
    int medIdxLidar = floor(copiedLidarTTC.size() / 2.0);
    int medIdxCamera = floor(copiedCameraTTC.size() / 2.0);
    double medTTCLidar =
        copiedLidarTTC.size() % 2 == 0
            ? (copiedLidarTTC[medIdxLidar - 1] + copiedLidarTTC[medIdxLidar]) /
                  2.0
            : copiedLidarTTC[medIdxLidar];
    double medTTCCamera = copiedCameraTTC.size() % 2 == 0
                              ? (copiedCameraTTC[medIdxCamera - 1] +
                                 copiedCameraTTC[medIdxCamera]) /
                                    2.0
                              : copiedCameraTTC[medIdxCamera];

    for (int i = 0; i < imgEndIndex; ++i) {
      summaryFile << config.detectorType << "," << config.descriptorType << ","
                  << i << "," << std::fixed << std::setprecision(8)
                  << config.LidarTTC[i] << "," << std::fixed
                  << std::setprecision(8) << config.cameraTTC[i] << endl;
    }

    comparisonFile << config.detectorType << "," << config.descriptorType << ","
                   << std::fixed << std::setprecision(8) << minTTCLidar << ","
                   << std::fixed << std::setprecision(8) << minTTCCamera << ","
                   << std::fixed << std::setprecision(8) << maxTTCLidar << ","
                   << std::fixed << std::setprecision(8) << maxTTCCamera << ","
                   << std::fixed << std::setprecision(8) << meanTTCLidar << ","
                   << std::fixed << std::setprecision(8) << meanTTCCamera << ","
                   << std::fixed << std::setprecision(8) << medTTCLidar << ","
                   << std::fixed << std::setprecision(8) << medTTCCamera << ","
                   << std::fixed << std::setprecision(8) << stdevTTCLidar << ","
                   << std::fixed << std::setprecision(8) << stdevTTCCamera
                   << endl;
    summaryFile << endl;
  }

  summaryFile.close();
  comparisonFile.close();

  return 0;
}
