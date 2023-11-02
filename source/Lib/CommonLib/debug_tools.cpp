//
// Created by Office on 10.02.21.
//

#include "debug_tools.h"

#ifdef OpenCV_FOUND
#define DEBUG_WITH_OPENCV 1

#include <opencv2/opencv.hpp>

#endif

namespace debug_tools {
  void showBuf(const Pel *buf, int stride, int width, int height, int bitdepth, const char *windowID, int delay, const char* windowTitle) {
#ifdef OpenCV_FOUND
#if DEBUG_WITH_OPENCV
    cv::Mat img;
    cv::Mat(height, width, CV_16UC1, (Pel *) buf, 2 * stride).copyTo(img);
    img.convertTo(img, CV_16UC1, 1.0 * (1 << (16 - bitdepth)), 0.0);
    cv::namedWindow(windowID, cv::WINDOW_KEEPRATIO);
    cv::imshow(windowID, img);
    if (windowTitle) {
      cv::setWindowTitle(windowID, windowTitle);
    }
    cv::waitKey(delay);
#endif
#endif
  }

  void showYUV(const PelUnitBuf &yuv, int bitdepth, const char *windowID, int delay, const char* windowTitle) {
#ifdef OpenCV_FOUND
#if DEBUG_WITH_OPENCV
    const Pel *yPlane = yuv.bufs[COMPONENT_Y].buf;
    const Pel *uPlane = yuv.bufs[COMPONENT_Cb].buf;
    const Pel *vPlane = yuv.bufs[COMPONENT_Cr].buf;

    const auto height = int(yuv.bufs[COMPONENT_Y].height);
    const auto width = int(yuv.bufs[COMPONENT_Y].width);

    // create cv::Mat objects for the Y, U, and V planes
    cv::Mat yImg, uImg, vImg;
    cv::Mat(height, width, CV_16UC1, (Pel *) yPlane).copyTo(yImg);
    cv::Mat(height / 2, width / 2, CV_16UC1, (Pel *) uPlane).copyTo(uImg);
    cv::Mat(height / 2, width / 2, CV_16UC1, (Pel *) vPlane).copyTo(vImg);

    // Scale according to bitdepth
    yImg.convertTo(yImg, CV_16UC1, 1.0 * (1 << (16 - bitdepth)), 0.0);
    uImg.convertTo(uImg, CV_16UC1, 1.0 * (1 << (16 - bitdepth)), 0.0);
    vImg.convertTo(vImg, CV_16UC1, 1.0 * (1 << (16 - bitdepth)), 0.0);

    // Resize U and V channels to match the size of the Y channel
    cv::Mat uImgResized, vImgResized;
    cv::resize(uImg, uImgResized, yImg.size(), 0, 0, cv::INTER_LINEAR);
    cv::resize(vImg, vImgResized, yImg.size(), 0, 0, cv::INTER_LINEAR);

    // Merge the resized Y, U, and V channels back into a single image
    std::vector<cv::Mat> mergedChannels = {yImg, uImgResized, vImgResized};
    cv::Mat mergedImg;
    cv::merge(mergedChannels, mergedImg);

    // Convert the YUV image to RGB
    cv::Mat rgbImg;
    cv::cvtColor(mergedImg, rgbImg, cv::COLOR_YUV2BGR);

    // display the RGB image
    cv::namedWindow(windowID, cv::WINDOW_KEEPRATIO);
    cv::imshow(windowID, rgbImg);
    if (windowTitle) {
      cv::setWindowTitle(windowID, windowTitle);
    }
    cv::waitKey(delay);
#endif
#endif
  }
}
