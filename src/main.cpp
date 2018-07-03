#include <iostream>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>


void calibrate_camera(const std::vector<cv::Mat>& /*images*/) {
//    cv::Mat camera_matrix;
}

int main() {
    cv::VideoCapture capture = cv::VideoCapture(0);
    if (!capture.isOpened()) {
        std::cout << "Cannot use the camera" << std::endl;
        return -1;
    }

    cv::namedWindow("ar", cv::WINDOW_AUTOSIZE);

    std::vector<cv::Mat> calibration_images;

    bool finished = false;
    while (!finished) {
        cv::Mat image;
        capture >> image;

        cv::Mat flipped_image;
        cv::flip(image, flipped_image, 1);

        cv::Size pattern_size(9, 6);
        std::vector<cv::Point2f> centers;
        bool found = cv::findChessboardCorners(
            flipped_image, pattern_size, centers,
            cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE | cv::CALIB_CB_FAST_CHECK);

        if (found) {
            cv::Mat image_with_chessboard;
            flipped_image.copyTo(image_with_chessboard);
            cv::drawChessboardCorners(image_with_chessboard, pattern_size, cv::Mat(centers), found);
            cv::imshow("ar", image_with_chessboard);
        } else {
            cv::imshow("ar", flipped_image);
        }

        char key = cv::waitKey(10);
        switch (key) { // ESC
        case ' ':
            if (found) {
                calibration_images.push_back(flipped_image);
                std::cout << "Calibration image " << calibration_images.size() << " collected\n";
            } else {
                std::cout << "Chessboard on image not found\n";
            }
            break;
        case 'c':
            if (calibration_images.size() >= 15) {
                calibrate_camera(calibration_images);
            } else {
                std::cout << "Not enough of calibration images (" << calibration_images.size()
                          << " < 15)\n";
            }
            break;
        case 27:
            finished = true;
            break;
        }
    }

    return 0;
}
