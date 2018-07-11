#include <iostream>
#include <vector>

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

/*cv::Mat calibrate_camera(const std::vector<cv::Mat>& images,
                         const std::vector<std::vector<cv::Point2f>>& centers) {
    cv::Mat camera_matrix = cv::Mat::eye(3, 3, CV_64F);
    cv::Mat distance_coefficients = Mat::zeros(8, 1, CV_64F);

    std::vector<std::vector<cv::Point3f>> world_space_corners;

    std::vector<cv::Mat> rotate_vectors;
    std::vector<cv::Mat> translate_vectors;

    return camera_matrix;
}*/

bool read_calibration_file(const std::string& file_name, cv::Mat& camera_matrix,
                           cv::Mat& distortion_coefficients) {
    cv::FileStorage camera_calibration_file(file_name, cv::FileStorage::READ);
    if (!camera_calibration_file.isOpened()) {
        std::cerr << "Cannot open calibration file: " << file_name << "\n";
        return false;
    }

    camera_calibration_file["camera_matrix"] >> camera_matrix;
    camera_calibration_file["distortion_coefficients"] >> distortion_coefficients;
    camera_calibration_file.release();

    return true;
}

void calcChessboardCorners(cv::Size boardSize, float squareSize,
                           std::vector<cv::Point3f>& corners) {
    for (int i = 0; i < boardSize.height; i++)
        for (int j = 0; j < boardSize.width; j++)
            corners.push_back(cv::Point3f(float(j * squareSize), float(i * squareSize), 0));
}

int main(int argc, char* argv[]) {
    const auto FRAME_WIDTH = 640;
    const auto FRAME_HEIGHT = 480;

    cv::VideoCapture capture = cv::VideoCapture(0);
    capture.set(cv::CAP_PROP_FRAME_WIDTH, FRAME_WIDTH);
    capture.set(cv::CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT);

    cv::Mat camera_matrix = cv::Mat::eye(3, 3, CV_64F);
    cv::Mat distortion_coefficients = cv::Mat::zeros(8, 1, CV_64F);

    bool params_file_read = false;
    if (argc == 2) {
        params_file_read = read_calibration_file(argv[1], camera_matrix, distortion_coefficients);
    }

    if (!capture.isOpened()) {
        std::cerr << "Cannot use the camera!\n";
        return -1;
    }

    cv::namedWindow("ar", cv::WINDOW_AUTOSIZE);

    std::vector<cv::Mat> calibration_images;
    std::vector<std::vector<cv::Point2f>> calibration_corners; // image_space_corners
    std::vector<std::vector<cv::Point3f>> known_corners;

    bool calibrated = false;

    bool finished = false;
    double tick_previous = cv::getTickCount();
    double tick_current = cv::getTickCount();
    while (!finished) {
        tick_current = cv::getTickCount();
        cv::Mat image;
        capture >> image;

        cv::Mat flipped_image;
        cv::flip(image, flipped_image, 1);

        const int corners_horizontally = 9;
        const int corners_vertically = 6;
        cv::Size pattern_size(corners_horizontally, corners_vertically);
        std::vector<cv::Point2f> corners;

        double t = (tick_current - tick_previous) / cv::getTickFrequency();

        bool found = false;
        if (t > 2.0) {
            found = cv::findChessboardCorners(flipped_image, pattern_size, corners,
                                              cv::CALIB_CB_ADAPTIVE_THRESH |
                                                  cv::CALIB_CB_NORMALIZE_IMAGE |
                                                  cv::CALIB_CB_FAST_CHECK);
        }

        std::vector<cv::Point3f> world_space_corners;
        for (auto i = 0; i < corners_horizontally; ++i) {
            for (auto j = 0; j < corners_vertically; ++j) {
                world_space_corners.push_back(cv::Point3f(i, j, 0.0f));
            }
        }

        if (found && !calibrated) {
            cv::Mat image_with_chessboard;
            flipped_image.copyTo(image_with_chessboard);

            cv::Mat gray_image;
            cv::cvtColor(flipped_image, gray_image, cv::COLOR_RGB2GRAY);
            cv::cornerSubPix(
                gray_image, corners, cv::Size(11, 11), cv::Size(-1, -1),
                cv::TermCriteria(cv::TermCriteria::EPS | cv::TermCriteria::COUNT, 30, 0.1));
            cv::drawChessboardCorners(image_with_chessboard, pattern_size, cv::Mat(corners), found);
            cv::imshow("ar", image_with_chessboard);

            calibration_images.push_back(flipped_image);
            calibration_corners.push_back(corners);
            known_corners.push_back(world_space_corners);
            std::cout << "Calibration image " << calibration_images.size() << " collected\n";
            tick_previous = cv::getTickCount();
        } else if (calibrated || params_file_read) {
            cv::Mat imageUndistorted;
            cv::undistort(flipped_image, imageUndistorted, camera_matrix, distortion_coefficients);
            cv::imshow("ar", imageUndistorted);
        } else {
            cv::imshow("ar", flipped_image);
        }

        const int MIN_NUMBER_OF_IMAGES = 3;

        char key = cv::waitKey(10);
        switch (key) {
        case ' ': // call try_store_corners()
            if (found) {
                calibration_images.push_back(flipped_image);
                calibration_corners.push_back(corners);
                known_corners.push_back(world_space_corners);
                std::cout << "Calibration image " << calibration_images.size() << " collected\n";
            } else {
                std::cout << "Chessboard on image not found\n";
            }
            break;
        case 'c': // call calibrate()
        {
            if (calibration_images.size() >= MIN_NUMBER_OF_IMAGES) {

                std::vector<cv::Mat> rvecs;
                std::vector<cv::Mat> tvecs;

                camera_matrix.at<double>(0, 0) = 1;

                std::vector<std::vector<cv::Point3f>> objectPoints(1);
                calcChessboardCorners(cv::Size(9, 6), 1.0, objectPoints[0]);
                objectPoints.resize(calibration_corners.size(), objectPoints[0]);

                double rms =
                    cv::calibrateCamera(objectPoints, calibration_corners, image.size(),
                                        camera_matrix, distortion_coefficients, rvecs, tvecs);
                std::cout << "calibration error: " << rms << "\n";
                // calibrate_camera(calibration_images, calibration_corners);

                calibrated = true;

                std::cout << "camera_matrix:\n";
                for (auto i = 0; i < 3; ++i) {
                    for (auto j = 0; j < 3; ++j) {
                        std::cout << camera_matrix.at<double>(i, j) << " ";
                    }
                    std::cout << "\n";
                }
                std::cout << "distortion_coefficients:\n";
                for (auto i = 0; i < 8; ++i) {
                    std::cout << distortion_coefficients.at<double>(i) << " ";
                }
                std::cout << "\n";
            } else {
                std::cout << "Not enough of calibration images (" << calibration_images.size()
                          << " < " << MIN_NUMBER_OF_IMAGES << ")\n";
            }

            cv::FileStorage camera_calibration_file("../data/camera_calibration",
                                                    cv::FileStorage::WRITE);
            camera_calibration_file << "camera_matrix" << camera_matrix;
            camera_calibration_file << "distortion_coefficients" << distortion_coefficients;
            camera_calibration_file.release();

            break;
        }
        case 27: // ESC
            finished = true;
            break;
        }
    }

    return 0;
}
