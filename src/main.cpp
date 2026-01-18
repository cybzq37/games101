#include <iostream>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>

int main(int argc, char* argv[])
{
    std::cout << "Games101 项目启动" << std::endl;
    std::cout << "OpenCV 版本: " << CV_VERSION << std::endl;

    // 测试 Eigen
    Eigen::MatrixXd A = Eigen::MatrixXd::Random(3, 3);
    std::cout << "\n随机 3x3 矩阵:\n" << A << std::endl;

    // 测试 OpenCV（可选）
    // cv::Mat image = cv::Mat::zeros(100, 100, CV_8UC3);
    // std::cout << "\nOpenCV 矩阵创建成功" << std::endl;

    return 0;
}
