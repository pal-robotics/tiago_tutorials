#ifndef FEATURES2D_FACTORY_H
#define FEATURES2D_FACTORY_H

#include <opencv2/features2d.hpp>
#ifndef NO_CV_XFEATURES2D
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>
#endif

CV_WRAP static cv::Ptr<cv::Feature2D> create(const std::string& detector_type)
{
  if (detector_type == "ORB")
    return cv::ORB::create();
  else if (detector_type == "BRISK")
    return cv::BRISK::create();
  else if (detector_type == "MSER")
    return cv::MSER::create();
  else if (detector_type == "GFTT")
    return cv::GFTTDetector::create();
#ifndef NO_CV_XFEATURES2D
  else if (detector_type == "SURF")
    return cv::xfeatures2d::SURF::create();
  else if (detector_type == "SIFT")
    return cv::xfeatures2d::SIFT::create();
  else if (detector_type == "STAR")
    return cv::xfeatures2d::StarDetector::create();
  else if (detector_type == "BRIEF")
    return cv::xfeatures2d::BriefDescriptorExtractor::create();
#endif
  else if (detector_type == "HARRIS")
  {
    cv::Ptr<cv::GFTTDetector> ptr = cv::GFTTDetector::create();
    ptr->setHarrisDetector(true);
    return ptr;
  }
  else if (detector_type == "SimpleBlob")
    return cv::SimpleBlobDetector::create();
  else if (detector_type == "FAST")
    return cv::FastFeatureDetector::create();
  else
    throw std::runtime_error("Unsupported type of detector " + detector_type);
}

#endif  // FEATURES2D_FACTORY_H
