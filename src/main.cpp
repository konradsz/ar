#include <iostream>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

int main() {
    cv::VideoCapture capture = cv::VideoCapture(0);
    if (!capture.isOpened()) {
        std::cout << "Cannot use the camera" << std::endl;
        return -1;
    }

    cv::namedWindow("ar", cv::WINDOW_AUTOSIZE);

    while (true) {
        // capture the image from camera
        cv::Mat image;
        capture >> image;

        // flip the image
        cv::Mat flipped_image;
        cv::flip(image, flipped_image, 1);

        // convert to grey scale
        cv::Mat gray_image;
        cv::cvtColor(flipped_image, gray_image, cv::COLOR_BGR2GRAY);

        cv::Size pattern_size(9, 6);
        std::vector<cv::Point2f> centers;
        //cv::resize(gray_image, gray_image, cv::Size(320, 240));
        bool found = cv::findChessboardCorners(gray_image, pattern_size, centers);
        cv::drawChessboardCorners(flipped_image, pattern_size, cv::Mat(centers), found);

        cv::imshow("ar", flipped_image);
        if (cv::waitKey(1) == 27) {
            break;
        }
    }

    return 0;
}
