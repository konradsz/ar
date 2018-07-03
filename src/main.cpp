#include <iostream>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

int main() {
    cv::VideoCapture capture = cv::VideoCapture(0);
    if (!capture.isOpened()) {
        std::cout << "Cannot use camera" << std::endl;
        return -1;
    }

    cv::namedWindow("ar", cv::WINDOW_AUTOSIZE);

    while (true) {
        cv::Mat image;
        cv::Mat flipped_image;
        capture >> image;
        cv::flip(image, flipped_image, 1);
        cv::imshow("ar", flipped_image);
        if (cv::waitKey(30) == 27) {
            break;
        }
    }

    /*cv::Size patternsize(9, 6); // number of centers
    std::vector<cv::Point2f> centers;

    // bool patternfound = findChessboardCorners(frame, patternsize, centers);
    bool found = cv::findChessboardCorners(image, patternsize, centers);
    if (found)
      std::cout << "found!" << std::endl;
    else
      std::cout << "not found!" << std::endl;
    cv::drawChessboardCorners(image2, patternsize, cv::Mat(centers), found);*/
    return 0;
}
