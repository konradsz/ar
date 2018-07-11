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
        std::cerr << "Cannot open calibration file: " << file_name << '\n';
        return false;
    }

    camera_calibration_file["camera_matrix"] >> camera_matrix;
    camera_calibration_file["distortion_coefficients"] >> distortion_coefficients;
    camera_calibration_file.release();

    return true;
}

void calcChessboardCorners(const cv::Size& board_size, float square_size,
                           std::vector<cv::Point3f>& corners) {
    for (int i = 0; i < board_size.height; ++i) {
        for (int j = 0; j < board_size.width; ++j) {
            corners.push_back(cv::Point3f(float(j * square_size), float(i * square_size), 0));
        }
    }
}

int main(int argc, char* argv[]) {
    if (argc != 2) {
        std::cerr << "No calibration file name passed!" << '\n';
        return -1;
    }

    const auto FRAME_WIDTH = 640;
    const auto FRAME_HEIGHT = 480;

    cv::VideoCapture capture = cv::VideoCapture(0);
    capture.set(cv::CAP_PROP_FRAME_WIDTH, FRAME_WIDTH);
    capture.set(cv::CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT);

    if (!capture.isOpened()) {
        std::cerr << "Cannot use the camera!\n";
        return -1;
    }

    cv::Mat camera_matrix = cv::Mat::eye(3, 3, CV_64F);
    cv::Mat distortion_coefficients = cv::Mat::zeros(8, 1, CV_64F);

    const std::string file_name = argv[1];
    bool calibrated = read_calibration_file(file_name, camera_matrix, distortion_coefficients);

    cv::namedWindow("ar", cv::WINDOW_AUTOSIZE);

    std::vector<std::vector<cv::Point2f>> calibration_corners;

    bool finished = false;
    double tick_previous = cv::getTickCount();
    double tick_current = cv::getTickCount();
    while (!finished) {
        tick_current = cv::getTickCount();
        cv::Mat frame;
        capture >> frame;

        cv::Mat flipped_frame;
        cv::flip(frame, flipped_frame, 1);

        const int corners_horizontally = 9;
        const int corners_vertically = 6;
        cv::Size pattern_size(corners_horizontally, corners_vertically);
        std::vector<cv::Point2f> corners;

        double t = (tick_current - tick_previous) / cv::getTickFrequency();

        bool found = false;
        if (t > 2.0) {
            found = cv::findChessboardCorners(flipped_frame, pattern_size, corners,
                                              cv::CALIB_CB_ADAPTIVE_THRESH |
                                                  cv::CALIB_CB_NORMALIZE_IMAGE |
                                                  cv::CALIB_CB_FAST_CHECK);
        }

        if (found && !calibrated) {
            cv::Mat frame_with_chessboard;
            flipped_frame.copyTo(frame_with_chessboard);

            cv::Mat gray_frame;
            cv::cvtColor(flipped_frame, gray_frame, cv::COLOR_RGB2GRAY);
            cv::cornerSubPix(
                gray_frame, corners, cv::Size(5, 5), cv::Size(-1, -1),
                cv::TermCriteria(cv::TermCriteria::EPS | cv::TermCriteria::COUNT, 30, 0.1));
            cv::drawChessboardCorners(frame_with_chessboard, pattern_size, cv::Mat(corners), found);
            cv::imshow("ar", frame_with_chessboard);

            calibration_corners.push_back(corners);
            std::cout << "Calibration corners " << calibration_corners.size() << " collected\n";
            tick_previous = cv::getTickCount();
        } else if (calibrated) {
            cv::Mat undistorted_frame;
            cv::undistort(flipped_frame, undistorted_frame, camera_matrix, distortion_coefficients);
            cv::imshow("ar", undistorted_frame);
        } else {
            cv::imshow("ar", flipped_frame);
        }

        const int MIN_NUMBER_OF_SAMPLES = 15;

        char key = cv::waitKey(10);
        switch (key) {
        case ' ': // call try_store_corners()
            if (found) {
                calibration_corners.push_back(corners);
                std::cout << "Calibration corners " << calibration_corners.size() << " collected\n";
            } else {
                std::cout << "Chessboard on image not found\n";
            }
            break;
        case 'c': // call calibrate()
        {
            if (calibration_corners.size() >= MIN_NUMBER_OF_SAMPLES) {
                std::vector<cv::Mat> rotation_vectors;
                std::vector<cv::Mat> translation_vectors;

                camera_matrix.at<double>(0, 0) = 1;

                std::vector<std::vector<cv::Point3f>> object_points(1);
                calcChessboardCorners(pattern_size, 1.0, object_points[0]);
                object_points.resize(calibration_corners.size(), object_points[0]);

                double rms = cv::calibrateCamera(object_points, calibration_corners, frame.size(),
                                                 camera_matrix, distortion_coefficients,
                                                 rotation_vectors, translation_vectors);
                std::cout << "calibration error: " << rms << '\n';
                // calibrate_camera(calibration_images, calibration_corners);

                calibrated = true;

                std::cout << "camera_matrix:\n";
                for (auto i = 0; i < 3; ++i) {
                    for (auto j = 0; j < 3; ++j) {
                        std::cout << camera_matrix.at<double>(i, j) << " ";
                    }
                    std::cout << '\n';
                }
                std::cout << "distortion_coefficients:\n";
                for (auto i = 0; i < 8; ++i) {
                    std::cout << distortion_coefficients.at<double>(i) << " ";
                }
                std::cout << '\n';
            } else {
                std::cout << "Not enough of samples (" << calibration_corners.size() << " < "
                          << MIN_NUMBER_OF_SAMPLES << ")\n";
            }

            cv::FileStorage camera_calibration_file(file_name, cv::FileStorage::WRITE);
            camera_calibration_file << "camera_matrix" << camera_matrix;
            camera_calibration_file << "distortion_coefficients" << distortion_coefficients;
            camera_calibration_file.release();

            std::cout << "Camera calibration file saved: " << file_name << '\n';
            break;
        }
        case 'q':
        case 27: // ESC
            finished = true;
            break;
        }
    }

    return 0;
}
