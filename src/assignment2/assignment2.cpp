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
#include <opencv2/core/cvdef.h>
#include <opencv2/opencv.hpp>

constexpr double MY_PI = 3.1415926;

/* 模型变换 */
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

/* 视口变换 */
Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
	Eigen::Matrix4f view = Eigen::Matrix4f::Identity();
	view.col(3) << -eye_pos[0], -eye_pos[1], -eye_pos[2], 1;
    return view;
}

/* 投影变换 */
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

int assignment2(int argc, const char** argv)
{
    float angle = 0;
    bool command_line = false;
    std::string filename = "output.png";

    if (argc == 2)
    {
        command_line = true;
        filename = std::string(argv[1]);
    }

    rst::Rasterizer r(700, 700);

    Eigen::Vector3f eye_pos = {0,0,5};


    std::vector<Eigen::Vector3f> pos
            {
                    {2, 0, -2},
                    {0, 2, -2},
                    {-2, 0, -2},
                    {3.5, -1, -5},
                    {2.5, 1.5, -5},
                    {-1, 0.5, -5}
            };

    std::vector<Eigen::Vector3i> ind
            {
                    {0, 1, 2},
                    {3, 4, 5}
            };

    std::vector<Eigen::Vector3f> cols
            {
                    {217.0, 238.0, 185.0},
                    {217.0, 238.0, 185.0},
                    {217.0, 238.0, 185.0},
                    {185.0, 217.0, 238.0},
                    {185.0, 217.0, 238.0},
                    {185.0, 217.0, 238.0}
            };

    auto pos_id = r.load_positions(pos);
    auto ind_id = r.load_indices(ind);
    auto col_id = r.load_colors(cols);

    int key = 0;
    int frame_count = 0;

    if (command_line)
    {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, col_id, rst::Primitive::Triangle);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::cvtColor(image, image, cv::COLOR_RGB2BGR);

        cv::imwrite(filename, image);

        return 0;
    }

    while(key != 27)
    {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, col_id, rst::Primitive::Triangle);

        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::cvtColor(image, image, cv::COLOR_RGB2BGR);
        cv::imshow("image", image);
        key = cv::waitKey(10);

        std::cout << "frame count: " << frame_count++ << '\n';
    }

    return 0;
}
// clang-format on
