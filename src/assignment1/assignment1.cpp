/**
 * Assignment 1: Transformations
 * 给定3个顶点 v0(2.0，2.0, -2.0), v1(0.0, 2.0, -2.0), v2(-2.0, 0.0, -2.0)
 * 变换为屏幕坐标并在屏幕上绘制对应的线框三角形
 * 模型 视图 投影 视口变换
 */

#include "Triangle.h"
#include "rasterizer.h"
#include <Eigen/Eigen>
#include <iostream>
#include <opencv2/opencv.hpp>

constexpr double MY_PI = 3.1415926;

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
	Eigen::Matrix4f view = Eigen::Matrix4f::Identity();
	view.col(3) << -eye_pos[0], -eye_pos[1], -eye_pos[2], 1;
    return view;
}

Eigen::Matrix4f get_model_matrix(float rotation_angle) // 旋转矩阵（绕z轴旋转）
{
	Eigen::Matrix4f model = Eigen::Matrix4f::Identity();
	float rad = rotation_angle / 180.0 * MY_PI;
	float sin = std::sin(rad);
	float cos = std::cos(rad);
	model.col(0) << cos, sin, 0, 0;
	model.col(1) << -sin, cos, 0, 0;
    return model;
}

Eigen::Matrix4f get_projection_matrix( // 构建透视矩阵
    float eye_fov,  // 垂直视野角
	float aspect_ratio, // 视口宽高比
	float zNear, // 近平面
	float zFar // 远平面
)
{
    float angel = eye_fov / 180.0 * MY_PI;
    float r, l, t, b, n, f;  // 右、左、上、下、近、远
	n = -zNear; // 注意这里的zNear和zFar是正数，代表距离，但在坐标系中是负数
	f = -zFar;
	t = abs(n) * tan(angel / 2); // top
	b = -t; // bottom
	r = t * aspect_ratio; // right
	l = -r; // left

	Eigen::Matrix4f m_persp_ortho = Eigen::Matrix4f::Identity();
    m_persp_ortho << n, 0, 0, 0,
             0, n, 0, 0,
             0, 0, n + f, -n * f,
		     0, 0, 1, 0;

	Eigen::Matrix4f m_ortho = Eigen::Matrix4f::Identity();
    m_ortho << 2 / (r - l), 0, 0, -(r + l) / 2,
               0, 2 / (t - b), 0, -(t + b) / 2,
               0, 0, 2 / (n - f), -(n + f) / 2,
		       0, 0, 0, 1;

	return m_ortho * m_persp_ortho;

    // Edit end
}


// 基于罗德里格斯旋转公式
Eigen::Matrix4f get_rotation(Vector3f axis, float angel)
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();
    //去掉长度，保留方向
    float norm = sqrt(axis[0]*axis[0]+axis[1]*axis[1]+axis[2]+axis[2]);
    axis[0]/=norm;
    axis[1]/=norm;
    axis[2]/=norm;

    //围绕任意轴旋转的矩阵公式
    float rad = angel / 180.0 * MY_PI;
    Eigen::Matrix3f n(3,3);
    n << 0, -axis[2], axis[1],
        axis[2], 0, -axis[0],
        -axis[1],axis[0],0;

    Eigen::Matrix3f component1 = Eigen::Matrix3f::Identity() * cos(rad);
    Eigen::Matrix3f component2 = axis * axis.transpose() * (1 - cos(rad));
    Eigen::Matrix3f component3 = n*sin(rad);

    Eigen::Matrix3f m_rotate = component1 + component2 + component3;

    Eigen::Matrix4f m4_rotate = Eigen::Matrix4f::Identity();
    //在0,0位置取3x3的矩阵
    m4_rotate.block(0,0,3,3) = m_rotate;

    model = m4_rotate * model;
    return model;
}
// Edit end

int run_assignment1(int argc, const char** argv)
{
    float angle = 0;
    bool command_line = false;
    std::string filename = "output.png";

    if (argc >= 3) {
        command_line = true;
        angle = std::stof(argv[2]); // -r by default
        if (argc == 4) {
            filename = std::string(argv[3]);
        }
        else
            return 0;
    }

    rst::Rasterizer r(700, 700);    // 创建光栅化器对象

    Eigen::Vector3f eye_pos = {0, 0, 5}; // 相机位置

    // Edit begin
    Eigen::Vector3f rotate_axis = {1,1,0}; // 旋转轴
    // Edit end

    std::vector<Eigen::Vector3f> pos{{2, 0, -2}, {0, 2, -2}, {-2, 0, -2}}; // 三角形顶点

    std::vector<Eigen::Vector3i> ind{{0, 1, 2}}; // 三角形索引

    auto pos_id = r.load_positions(pos); // 加载顶点位置
    auto ind_id = r.load_indices(ind); // 加载顶点索引

    int key = 0;
    int frame_count = 0;

    if (command_line) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        // Edit begin
        //围绕z轴旋转
        //r.set_model(get_model_matrix(angle));
        //围绕任意轴旋转
        r.set_model(get_rotation(rotate_axis,angle));
        // Edit end

        r.set_view(get_view_matrix(eye_pos));
        //注意这里写入的zNear和zFar是正数，代表着距离，但课程上推导的透视矩阵是坐标，且假定是朝向z负半轴的，所以透视矩阵是需要取反的
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);

        cv::imwrite(filename, image);

        return 0;
    }

    while (key != 27) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        // Edit begin
        //围绕z轴旋转
        //r.set_model(get_model_matrix(angle));
        //围绕任意轴旋转
        r.set_model(get_rotation(rotate_axis,angle));
        // Edit end

        r.set_view(get_view_matrix(eye_pos));
        //注意这里写入的zNear和zFar是正数，代表着距离，但课程上推导的透视矩阵是坐标，且假定是朝向z负半轴的，所以透视矩阵是需要取反的
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);

        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::imshow("image", image);
        key = cv::waitKey(10);

        std::cout << "frame count: " << frame_count++ << '\n';

        if (key == 'a') {
            angle += 10;
        }
        else if (key == 'd') {
            angle -= 10;
        }
    }

    return 0;
}
