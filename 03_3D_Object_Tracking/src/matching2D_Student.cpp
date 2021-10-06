
#include "matching2D.hpp"
#include <numeric>

using namespace std;

// Find best matches for keypoints in two camera images based on several
// matching methods
void matchDescriptors(std::vector<cv::KeyPoint> &kPtsSource,
                      std::vector<cv::KeyPoint> &kPtsRef, cv::Mat &descSource,
                      cv::Mat &descRef, std::vector<cv::DMatch> &matches,
                      std::string descriptorType, std::string matcherType,
                      std::string selectorType, Configuaration &config) {
  // configure matcher
  bool crossCheck = false;
  cv::Ptr<cv::DescriptorMatcher> matcher;

  if (matcherType.compare("MAT_BF") == 0) {
    int normType = descriptorType.compare("DES_BINARY") == 0 ? cv::NORM_HAMMING
                                                             : cv::NORM_L2;
    matcher = cv::BFMatcher::create(normType, crossCheck);
    std::cout << "BF matching cross-check = " << crossCheck << std::endl;
  } else if (matcherType.compare("MAT_FLANN") == 0) {
    if (descSource.type() != CV_32F) { // OpenCV bug workaround : convert binary
                                       // descriptors to floating point due to a
                                       // bug in current OpenCV implementation
      descSource.convertTo(descSource, CV_32F);
    }
    if (descRef.type() != CV_32F) {
      descRef.convertTo(descRef, CV_32F);
    }

    matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::FLANNBASED);
    std::cout << "FLANN matching" << std::endl;
  }

  // perform matching task
  if (selectorType.compare("SEL_NN") == 0) {
    // nearest neighbor (best match)
    // Finds the best match for each descriptor in desc1
    double t = static_cast<double>(cv::getTickCount());

    // Finds the best match for each descriptor in desc1
    matcher->match(descSource, descRef, matches);

    t = (static_cast<double>(cv::getTickCount()) - t) / cv::getTickFrequency();
    std::cout << " (NN) with n=" << matches.size() << " matches in "
              << 1000 * t / 1.0 << " ms" << std::endl;
    config.numOfMatches.push_back(matches.size());
    config.durationDescriptorMatch.push_back(1000 * t / 1.0);
  } else if (selectorType.compare("SEL_KNN") == 0) {
    // k nearest neighbors (k=2)
    std::vector<std::vector<cv::DMatch>> knn_matches;
    double t = static_cast<double>(cv::getTickCount());

    // finds the 2 best matches
    matcher->knnMatch(descSource, descRef, knn_matches, 2);

    t = (static_cast<double>(cv::getTickCount()) - t) / cv::getTickFrequency();
    std::cout << " (KNN) with n=" << knn_matches.size() << " matches in "
              << 1000 * t / 1.0 << " ms" << std::endl;
    config.numOfMatches.push_back(knn_matches.size());
    config.durationDescriptorMatch.push_back(1000 * t / 1.0);

    // filter matches using descriptor distance ratio test
    double minDescDistRatio = 0.8;
    for (auto it = knn_matches.begin(); it != knn_matches.end(); ++it) {
      if ((*it)[0].distance < minDescDistRatio * (*it)[1].distance) {
        matches.push_back((*it)[0]);
      }
    }
    std::cout << "# keypoints removed = " << knn_matches.size() - matches.size()
              << std::endl;
  }
}

// Use one of several types of state-of-art descriptors to uniquely identify
// keypoints
void descKeypoints(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img,
                   cv::Mat &descriptors, std::string descriptorType,
                   Configuaration &config) {
  // select appropriate descriptor
  cv::Ptr<cv::DescriptorExtractor> extractor;
  if (descriptorType.compare("BRISK") == 0) {
    // FAST/AGAST detection threshold score.
    int threshold = 30;
    // detection octaves (use 0 to do single scale)
    int octaves = 3;
    // apply this scale to the pattern used for sampling the neighbourhood of a
    // keypoint.
    float patternScale = 1.0f;

    extractor = cv::BRISK::create(threshold, octaves, patternScale);
  } else if (descriptorType.compare("BRIEF") == 0) {
    extractor = cv::xfeatures2d::BriefDescriptorExtractor::create();
  } else if (descriptorType.compare("ORB") == 0) {
    extractor = cv::ORB::create();
  } else if (descriptorType.compare("FREAK") == 0) {
    extractor = cv::xfeatures2d::FREAK::create();
  } else if (descriptorType.compare("AKAZE") == 0) {
    extractor = cv::AKAZE::create();
  } else if (descriptorType.compare("SIFT") == 0) {
    extractor = cv::xfeatures2d::SiftDescriptorExtractor::create();
  } else {
    throw std::invalid_argument("Given descriptor type is invalid - " +
                                descriptorType);
  }

  // perform feature description
  double t = static_cast<double>(cv::getTickCount());
  extractor->compute(img, keypoints, descriptors);
  t = (static_cast<double>(cv::getTickCount()) - t) / cv::getTickFrequency();
  std::cout << descriptorType << " descriptor extraction in " << 1000 * t / 1.0
            << " ms" << std::endl;
  config.durationDescriptorExtraction.push_back(1000 * t / 1.0);
}

// Detect keypoints in image using the traditional Shi-Thomasi detector
void detKeypointsShiTomasi(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img,
                           Configuaration &config, bool bVis) {
  // compute detector parameters based on image size
  //  size of an average block for computing a derivative covariation matrix
  //  over each pixel neighborhood
  int blockSize = 4;
  // max. permissible overlap between two features in %
  double maxOverlap = 0.0;
  double minDistance = (1.0 - maxOverlap) * blockSize;
  // max. num. of keypoints
  int maxCorners = img.rows * img.cols / std::max(1.0, minDistance);
  // minimal accepted quality of image corners
  double qualityLevel = 0.01;
  double k = 0.04;

  // Apply corner detection
  double t = static_cast<double>(cv::getTickCount());
  std::vector<cv::Point2f> corners;
  cv::goodFeaturesToTrack(img, corners, maxCorners, qualityLevel, minDistance,
                          cv::Mat(), blockSize, false, k);

  // add corners to result vector
  for (auto it = corners.begin(); it != corners.end(); ++it) {
    cv::KeyPoint newKeyPoint;
    newKeyPoint.pt = cv::Point2f((*it).x, (*it).y);
    newKeyPoint.size = blockSize;
    keypoints.push_back(newKeyPoint);
  }
  t = (static_cast<double>(cv::getTickCount()) - t) / cv::getTickFrequency();
  std::cout << "Shi-Tomasi detection with n=" << keypoints.size()
            << " keypoints in " << 1000 * t / 1.0 << " ms" << std::endl;

  config.numOfKeypoints.push_back(keypoints.size());
  config.durationDetection.push_back(1000 * t / 1.0);

  // visualize results
  if (bVis) {
    cv::Mat visImage = img.clone();
    cv::drawKeypoints(img, keypoints, visImage, cv::Scalar::all(-1),
                      cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    std::string windowName = "Shi-Tomasi Corner Detector Results";
    cv::namedWindow(windowName, 6);
    imshow(windowName, visImage);
    cv::waitKey(0);
  }
}

void detKeypointsHarris(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img,
                        Configuaration &config, bool bVis) {
  // Detector parameters
  int blockSize = 2;
  int apertureSize = 3;
  int minResponse = 100;
  double k = 0.04;

  // Detect Harris corners and normalize output
  cv::Mat dst, dst_norm, dst_norm_scaled;
  dst = cv::Mat::zeros(img.size(), CV_32FC1);
  cv::cornerHarris(img, dst, blockSize, apertureSize, k, cv::BORDER_DEFAULT);
  cv::normalize(dst, dst_norm, 0, 255, cv::NORM_MINMAX, CV_32FC1, cv::Mat());
  cv::convertScaleAbs(dst_norm, dst_norm_scaled);

  // visualize results
  if (bVis) {
    std::string windowName = "Harris Corner Detector Response Matrix";
    cv::namedWindow(windowName, 4);
    cv::imshow(windowName, dst_norm_scaled);
    cv::waitKey(0);
  }

  // locate local maxima in the Harris response matrix and perform a non-maximum
  // suppression (NMS) in a local neighborhood around each maximum. The
  // resulting coordinates shall be stored in a list of keypoints of the type
  // `vector<cv::KeyPoint>`.
  double max_overlap = 0.0;

  double t = static_cast<double>(cv::getTickCount());

  for (int row = 0; row < dst_norm.rows; ++row) {
    for (int col = 0; col < dst_norm.cols; ++col) {
      int response = static_cast<int>(dst_norm.at<float>(row, col));
      // only store points over a threshold
      if (response > minResponse) {
        cv::KeyPoint new_keypoint;
        new_keypoint.pt = cv::Point2f(col, row);
        new_keypoint.size = 2 * apertureSize;
        new_keypoint.response = response;

        // perform a non-maximum suppression (NMS) in a local neighborhood
        // around
        bool overlap_tag = false;
        for (auto it = keypoints.begin(); it != keypoints.end(); ++it) {
          double keypoint_overlap = cv::KeyPoint::overlap(new_keypoint, *it);
          if (keypoint_overlap > max_overlap) {
            overlap_tag = true;
            if (new_keypoint.response > (*it).response) {
              *it = new_keypoint;
              break;
            }
          }
        }
        if (!overlap_tag) {
          keypoints.push_back(new_keypoint);
        }
      }
    }
  }

  t = (static_cast<double>(cv::getTickCount()) - t) / cv::getTickFrequency();
  std::cout << "Harris Detector detection with n=" << keypoints.size()
            << " keypoints in " << 1000 * t / 1.0 << " ms" << std::endl;
  config.numOfKeypoints.push_back(keypoints.size());
  config.durationDetection.push_back(1000 * t / 1.0);

  if (bVis) {
    std::string windowName = "Harris Corner Detector Results";
    cv::namedWindow(windowName, 5);
    cv::Mat vis_image = dst_norm_scaled.clone();
    cv::drawKeypoints(dst_norm_scaled, keypoints, vis_image,
                      cv::Scalar::all(-1),
                      cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    cv::imshow(windowName, vis_image);
    cv::waitKey(0);
  }
}

void detKeypointsModern(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img,
                        std::string detectorType, Configuaration &config,
                        bool bVis) {
  std::string windowName;
  cv::Ptr<cv::FeatureDetector> detector;
  int thresdhold = 30;

  if (detectorType.compare("FAST") == 0) {
    windowName = "FAST Result";

    detector = cv::FastFeatureDetector::create(thresdhold);
    double t = static_cast<double>(cv::getTickCount());

    // perform detection
    detector->detect(img, keypoints);

    t = (static_cast<double>(cv::getTickCount()) - t) / cv::getTickFrequency();
    std::cout << "FAST Detector detection with n=" << keypoints.size()
              << " keypoints in " << 1000 * t / 1.0 << " ms" << std::endl;
    config.numOfKeypoints.push_back(keypoints.size());
    config.durationDetection.push_back(1000 * t / 1.0);
  } else if (detectorType.compare("BRISK") == 0) {
    windowName = "BRISK Result";

    detector = cv::BRISK::create(thresdhold);
    double t = static_cast<double>(cv::getTickCount());

    // perform detection
    detector->detect(img, keypoints);

    t = (static_cast<double>(cv::getTickCount()) - t) / cv::getTickFrequency();
    std::cout << "BRISK Detector detection with n=" << keypoints.size()
              << " keypoints in " << 1000 * t / 1.0 << " ms" << std::endl;
    config.numOfKeypoints.push_back(keypoints.size());
    config.durationDetection.push_back(1000 * t / 1.0);
  } else if (detectorType.compare("ORB") == 0) {
    windowName = "ORB Result";

    detector = cv::ORB::create();
    double t = static_cast<double>(cv::getTickCount());

    // perform detection
    detector->detect(img, keypoints);
    t = (static_cast<double>(cv::getTickCount()) - t) / cv::getTickFrequency();
    std::cout << "ORB Detector detection with n=" << keypoints.size()
              << " keypoints in " << 1000 * t / 1.0 << " ms" << std::endl;
    config.numOfKeypoints.push_back(keypoints.size());
    config.durationDetection.push_back(1000 * t / 1.0);
  } else if (detectorType.compare("AKAZE") == 0) {
    windowName = "AKAZE Result";
    detector = cv::AKAZE::create();
    double t = static_cast<double>(cv::getTickCount());

    // perform detection
    detector->detect(img, keypoints);
    t = (static_cast<double>(cv::getTickCount()) - t) / cv::getTickFrequency();
    std::cout << "AKAZE Detector detection with n=" << keypoints.size()
              << " keypoints in " << 1000 * t / 1.0 << " ms" << std::endl;
    config.numOfKeypoints.push_back(keypoints.size());
    config.durationDetection.push_back(1000 * t / 1.0);
  } else if (detectorType.compare("SIFT") == 0) {
    windowName = "SIFT Result";
    detector = cv::xfeatures2d::SIFT::create();
    double t = static_cast<double>(cv::getTickCount());

    // perform detection
    detector->detect(img, keypoints);
    t = (static_cast<double>(cv::getTickCount()) - t) / cv::getTickFrequency();
    std::cout << "SIFT Detector detection with n=" << keypoints.size()
              << " keypoints in " << 1000 * t / 1.0 << " ms" << std::endl;
    config.numOfKeypoints.push_back(keypoints.size());
    config.durationDetection.push_back(1000 * t / 1.0);
  }

  if (bVis) {
    cv::Mat visImage = img.clone();
    cv::drawKeypoints(img, keypoints, visImage, cv::Scalar::all(-1),
                      cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    cv::namedWindow(windowName, 6);
    imshow(windowName, visImage);
    cv::waitKey(0);
  }
}