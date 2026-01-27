/**
 * Assignment 1: Transformations
 * 给定3个顶点 v0(2.0，2.0, -2.0), v1(0.0, 2.0, -2.0), v2(-2.0, 0.0, -2.0)
 * 变换为屏幕坐标并在屏幕上绘制对应的线框三角形
 * 模型 视图 投影 视口变换
 */

#include <iostream>
#include <Eigen/Eigen>
#include <opencv2/opencv.hpp>
#include <opencv2/core/cvdef.h>

#include "global.hpp"
#include "rasterizer.h"
#include "Shader.h"
#include "Texture.h"
#include "Triangle.h"
#include "OBJ_Loader.h"

/* 模型变换 */
Eigen::Matrix4f get_model_matrix(float rotation_angle) // 旋转矩阵（绕z轴旋转）
{
    Eigen::Matrix4f rotation;
    float angle = rotation_angle * MY_PI / 180.f;
    rotation << cos(angle), 0, sin(angle), 0,
                0, 1, 0, 0,
                -sin(angle), 0, cos(angle), 0,
                0, 0, 0, 1;

    Eigen::Matrix4f scale;
    scale << 2.5, 0, 0, 0,
            0, 2.5, 0, 0,
            0, 0, 2.5, 0,
            0, 0, 0, 1;

    Eigen::Matrix4f translate;
    translate << 1, 0, 0, 0,
                0, 1, 0, 0,
                0, 0, 1, 0,
                0, 0, 0, 1;

    return translate * rotation * scale;
}

/* 视口变换 */
Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
  Eigen::Matrix4f view = Eigen::Matrix4f::Identity();
  Eigen::Matrix4f translate;
  translate << 1, 0, 0, -eye_pos[0],
               0, 1, 0, -eye_pos[1],
               0, 0, 1, -eye_pos[2],
               0, 0, 0, 1;
  view = translate * view;
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


Eigen::Vector3f vertex_shader(const vertex_shader_payload& payload)
{
    return payload.position;
}

// 查看各处的法线方向和颜色值，用于调试
Eigen::Vector3f normal_fragment_shader(const fragment_shader_payload& payload)
{
    Eigen::Vector3f return_color = (payload.normal.head<3>().normalized() + Eigen::Vector3f(1.0f, 1.0f, 1.0f)) / 2.f;
    Eigen::Vector3f result;
    result << return_color.x() * 255, return_color.y() * 255, return_color.z() * 255;
    return result;
}

// 高光反射计算
static Eigen::Vector3f reflect(const Eigen::Vector3f& vec, const Eigen::Vector3f& axis)
{
    auto costheta = vec.dot(axis);
    return (2 * costheta * axis - vec).normalized();
}

struct light
{
    Eigen::Vector3f position;
    Eigen::Vector3f intensity;
};


// Phong 片段着色器（实际使用的是 Blinn-Phong 模型）
// 实现基于 Blinn-Phong 光照模型，计算每个片段的最终颜色
Eigen::Vector3f phong_fragment_shader(const fragment_shader_payload& payload)
{
    // 材质属性系数（RGB 三个通道）
    Eigen::Vector3f ka = Eigen::Vector3f(0.005, 0.005, 0.005);  // 环境光反射系数（ambient coefficient）
    Eigen::Vector3f kd = payload.color;                          // 漫反射系数（diffuse coefficient），使用片段颜色
    Eigen::Vector3f ks = Eigen::Vector3f(0.7937, 0.7937, 0.7937); // 镜面反射系数（specular coefficient）

    // 定义两个点光源：{位置, 强度}
    auto l1 = light{ {20, 20, 20}, {500, 500, 500} };   // 光源1：位置(20,20,20)，强度(500,500,500)
    auto l2 = light{ {-20, 20, 0}, {500, 500, 500} };   // 光源2：位置(-20,20,0)，强度(500,500,500)

    std::vector<light> lights = { l1, l2 };              // 光源列表
    Eigen::Vector3f amb_light_intensity{ 10, 10, 10 };    // 环境光强度
    Eigen::Vector3f eye_pos{ 0, 0, 10 };                  // 相机/观察者位置

    float p = 150;  // 镜面反射高光的衰减指数（shininess），值越大高光越集中

    // 从 payload 中获取片段信息
    Eigen::Vector3f color = payload.color;    // 片段颜色
    Eigen::Vector3f point = payload.view_pos; // 片段在视图空间中的位置
    Eigen::Vector3f normal = payload.normal;  // 片段法线向量

    Eigen::Vector3f result_color = { 0, 0, 0 };  // 初始化最终颜色为黑色

    // 遍历所有光源，累加每个光源的贡献
    for (auto& light : lights)
    {
        // 计算关键方向向量（单位向量）
        Eigen::Vector3f light_dir = (light.position - point).normalized();  // 从片段指向光源的方向
        Eigen::Vector3f view_dir = (eye_pos - point).normalized();          // 从片段指向观察者的方向
        Eigen::Vector3f half_dir = (light_dir + view_dir).normalized();     // 半角向量（Blinn-Phong 使用）

        // ========== 环境光分量 (Ambient) ==========
        // La = ka * I_ambient
        // 环境光不依赖于视角和光源方向，提供基础照明
        Eigen::Vector3f La = ka.cwiseProduct(amb_light_intensity); // 逐元素相乘（各着色点的环境光都是一样的）

        // ========== 计算距离衰减 ==========
        // 根据平方反比定律计算光强衰减：I' = I / r²
        float r2 = (light.position - point).dot(light.position - point);  // 距离的平方
        Eigen::Vector3f I_r2 = light.intensity / r2;                      // 衰减后的光强

        // ========== 漫反射分量 (Diffuse) ==========
        // Ld = kd * (I / r²) * max(0, N·L)
        // 漫反射遵循 Lambert 定律，与法线和光源方向的夹角余弦成正比
        Eigen::Vector3f Ld = kd.cwiseProduct(I_r2);  // 基础漫反射颜色
        Ld *= std::max(0.0f, normal.normalized().dot(light_dir));  // 乘以法线与光源方向的点积（Lambert 项）

        // ========== 镜面反射分量 (Specular) ==========
        // Ls = ks * (I / r²) * max(0, N·H)^p
        // 使用 Blinn-Phong 模型：使用半角向量 H 代替反射向量 R，计算更高效
        // p 是衰减指数，控制高光的集中程度
        Eigen::Vector3f Ls = ks.cwiseProduct(I_r2);  // 基础镜面反射颜色
        Ls *= std::pow(std::max(0.0f, normal.normalized().dot(half_dir)), p);  // 半角向量的点积的 p 次方

        // ========== 累加所有光照分量 ==========
        // 最终颜色 = 环境光 + 漫反射 + 镜面反射
        result_color += (La + Ld + Ls);
    }

    // 将颜色值从 [0, 1] 范围转换到 [0, 255] 范围并返回
    return result_color * 255.f;
}

Eigen::Vector3f texture_fragment_shader(const fragment_shader_payload& payload)
{
    Eigen::Vector3f return_color = { 0, 0, 0 };
    if (payload.texture)
    {
        // TODO: Get the texture value at the texture coordinates of the current fragment
        return_color = payload.texture->getColor(payload.tex_coords.x(), payload.tex_coords.y());
    }
    Eigen::Vector3f texture_color;
    texture_color << return_color.x(), return_color.y(), return_color.z();

    Eigen::Vector3f ka = Eigen::Vector3f(0.005, 0.005, 0.005);
    Eigen::Vector3f kd = texture_color / 255.f;
    Eigen::Vector3f ks = Eigen::Vector3f(0.7937, 0.7937, 0.7937);

    auto l1 = light{ {20, 20, 20}, {500, 500, 500} };
    auto l2 = light{ {-20, 20, 0}, {500, 500, 500} };

    std::vector<light> lights = { l1, l2 };
    Eigen::Vector3f amb_light_intensity{ 10, 10, 10 };
    Eigen::Vector3f eye_pos{ 0, 0, 10 };

    float p = 150;

    Eigen::Vector3f color = texture_color;
    Eigen::Vector3f point = payload.view_pos;
    Eigen::Vector3f normal = payload.normal;

    Eigen::Vector3f result_color = { 0, 0, 0 };

    for (auto& light : lights)
    {
        // TODO: For each light source in the code, calculate what the *ambient*, *diffuse*, and *specular*
        // components are. Then, accumulate that result on the *result_color* object.

        Eigen::Vector3f light_dir = (light.position - point).normalized();
        Eigen::Vector3f view_dir = (eye_pos - point).normalized();
        Eigen::Vector3f half_dir = (light_dir + view_dir).normalized();

        Eigen::Vector3f La = ka.cwiseProduct(amb_light_intensity);

        float r2 = (light.position - point).dot(light.position - point);
        Eigen::Vector3f I_r2 = light.intensity / r2;

        Eigen::Vector3f Ld = kd.cwiseProduct(I_r2);
        Ld *= std::max(0.0f, normal.normalized().dot(light_dir));

        Eigen::Vector3f Ls = ks.cwiseProduct(I_r2);
        Ls *= std::pow(std::max(0.0f, normal.normalized().dot(half_dir)), p);

        result_color += (La + Ld + Ls);
    }

    return result_color * 255.f;
}

Eigen::Vector3f displacement_fragment_shader(const fragment_shader_payload& payload)
{

    Eigen::Vector3f ka = Eigen::Vector3f(0.005, 0.005, 0.005);
    Eigen::Vector3f kd = payload.color;
    Eigen::Vector3f ks = Eigen::Vector3f(0.7937, 0.7937, 0.7937);

    auto l1 = light{ {20, 20, 20}, {500, 500, 500} };
    auto l2 = light{ {-20, 20, 0}, {500, 500, 500} };

    std::vector<light> lights = { l1, l2 };
    Eigen::Vector3f amb_light_intensity{ 10, 10, 10 };
    Eigen::Vector3f eye_pos{ 0, 0, 10 };

    float p = 150;

    Eigen::Vector3f color = payload.color;
    Eigen::Vector3f point = payload.view_pos;
    Eigen::Vector3f normal = payload.normal;

    float kh = 0.2, kn = 0.1;

    // TODO: Implement displacement mapping here
    // Let n = normal = (x, y, z)
    // Vector t = (x*y/sqrt(x*x+z*z),sqrt(x*x+z*z),z*y/sqrt(x*x+z*z))
    // Vector b = n cross product t
    // Matrix TBN = [t b n]
    // dU = kh * kn * (h(u+1/w,v)-h(u,v))
    // dV = kh * kn * (h(u,v+1/h)-h(u,v))
    // Vector ln = (-dU, -dV, 1)
    // Position p = p + kn * n * h(u,v)
    // Normal n = normalize(TBN * ln)

    float x = normal.x();
    float y = normal.y();
    float z = normal.z();

    Eigen::Vector3f t = Eigen::Vector3f(x * y / std::sqrt(x * x + z * z), std::sqrt(x * x + z * z), z * y / std::sqrt(x * x + z * z));
    Eigen::Vector3f b = normal.cross(t);

    Eigen::Matrix3f TBN;
    TBN << t.x(), b.x(), normal.x(),
        t.y(), b.y(), normal.y(),
        t.z(), b.z(), normal.z();

    float u = payload.tex_coords.x();
    float v = payload.tex_coords.y();
    float w = payload.texture->width;
    float h = payload.texture->height;

    float dU = kh * kn * (payload.texture->getColor(u + 1.0f / w, v).norm() - payload.texture->getColor(u, v).norm());
    float dV = kh * kn * (payload.texture->getColor(u, v + 1.0f / h).norm() - payload.texture->getColor(u, v).norm());

    Eigen::Vector3f ln = Eigen::Vector3f(-dU, -dV, 1.0f);

    point += (kn * normal * payload.texture->getColor(u, v).norm());

    normal = (TBN * ln).normalized();

    Eigen::Vector3f result_color = { 0, 0, 0 };

    for (auto& light : lights)
    {
        // TODO: For each light source in the code, calculate what the *ambient*, *diffuse*, and *specular*
        // components are. Then, accumulate that result on the *result_color* object.

        Eigen::Vector3f light_dir = (light.position - point).normalized();
        Eigen::Vector3f view_dir = (eye_pos - point).normalized();
        Eigen::Vector3f half_dir = (light_dir + view_dir).normalized();

        Eigen::Vector3f La = ka.cwiseProduct(amb_light_intensity);

        float r2 = (light.position - point).dot(light.position - point);
        Eigen::Vector3f I_r2 = light.intensity / r2;

        Eigen::Vector3f Ld = kd.cwiseProduct(I_r2);
        Ld *= std::max(0.0f, normal.normalized().dot(light_dir));

        Eigen::Vector3f Ls = ks.cwiseProduct(I_r2);
        Ls *= std::pow(std::max(0.0f, normal.normalized().dot(half_dir)), p);

        result_color += (La + Ld + Ls);
    }

    return result_color * 255.f;
}

Eigen::Vector3f bump_fragment_shader(const fragment_shader_payload& payload)
{

    Eigen::Vector3f ka = Eigen::Vector3f(0.005, 0.005, 0.005);
    Eigen::Vector3f kd = payload.color;
    Eigen::Vector3f ks = Eigen::Vector3f(0.7937, 0.7937, 0.7937);

    auto l1 = light{ {20, 20, 20}, {500, 500, 500} };
    auto l2 = light{ {-20, 20, 0}, {500, 500, 500} };

    std::vector<light> lights = { l1, l2 };
    Eigen::Vector3f amb_light_intensity{ 10, 10, 10 };
    Eigen::Vector3f eye_pos{ 0, 0, 10 };

    float p = 150;

    Eigen::Vector3f color = payload.color;
    Eigen::Vector3f point = payload.view_pos;
    Eigen::Vector3f normal = payload.normal;

    float kh = 0.2, kn = 0.1;

    // TODO: Implement bump mapping here
    // Let n = normal = (x, y, z)
    // Vector t = (x*y/sqrt(x*x+z*z),sqrt(x*x+z*z),z*y/sqrt(x*x+z*z))
    // Vector b = n cross product t
    // Matrix TBN = [t b n]
    // dU = kh * kn * (h(u+1/w,v)-h(u,v))
    // dV = kh * kn * (h(u,v+1/h)-h(u,v))
    // Vector ln = (-dU, -dV, 1)
    // Normal n = normalize(TBN * ln)

    float x = normal.x();
    float y = normal.y();
    float z = normal.z();

    Eigen::Vector3f t = Eigen::Vector3f(x * y / std::sqrt(x * x + z * z), std::sqrt(x * x + z * z), z * y / std::sqrt(x * x + z * z));
    Eigen::Vector3f b = normal.cross(t);

    Eigen::Matrix3f TBN;
    TBN << t.x(), b.x(), normal.x(),
        t.y(), b.y(), normal.y(),
        t.z(), b.z(), normal.z();

    float u = payload.tex_coords.x();
    float v = payload.tex_coords.y();
    float w = payload.texture->width;
    float h = payload.texture->height;

    float dU = kh * kn * (payload.texture->getColor(u + 1.0f / w, v).norm() - payload.texture->getColor(u, v).norm());
    float dV = kh * kn * (payload.texture->getColor(u, v + 1.0f / h).norm() - payload.texture->getColor(u, v).norm());

    Eigen::Vector3f ln = Eigen::Vector3f(-dU, -dV, 1.0f);
    normal = TBN * ln;

    Eigen::Vector3f result_color = { 0, 0, 0 };
    result_color = normal.normalized();

    return result_color * 255.f;
}

int assignment2(int argc, const char** argv)
{
    std::vector<Triangle *> TriangleList;

    float angle = 140.0;
    bool command_line = false;

    std::string filename = "output.png";
    objl::Loader Loader;
    std::string obj_path = "../models/spot/";

    // Load .obj File
    bool loadout = Loader.LoadFile("../models/spot/spot_triangulated_good.obj");
    for (auto mesh : Loader.LoadedMeshes)
    {
        for (int i = 0; i < mesh.Vertices.size(); i += 3)
        {
            Triangle *t = new Triangle();
            for (int j = 0; j < 3; j++)
            {
                t->setVertex(j, Vector4f(mesh.Vertices[i + j].Position.X, mesh.Vertices[i + j].Position.Y, mesh.Vertices[i + j].Position.Z, 1.0));
                t->setNormal(j, Vector3f(mesh.Vertices[i + j].Normal.X, mesh.Vertices[i + j].Normal.Y, mesh.Vertices[i + j].Normal.Z));
                t->setTexCoord(j, Vector2f(mesh.Vertices[i + j].TextureCoordinate.X, mesh.Vertices[i + j].TextureCoordinate.Y));
            }
            TriangleList.push_back(t);
        }
    }

    rst::rasterizer r(700, 700);

    auto texture_path = "hmap.jpg";
    r.set_texture(Texture(obj_path + texture_path));

    std::function<Eigen::Vector3f(fragment_shader_payload)> active_shader = phong_fragment_shader;

    if (argc >= 2)
    {
        command_line = true;
        filename = std::string(argv[1]);

        if (argc == 3 && std::string(argv[2]) == "texture")
        {
            std::cout << "Rasterizing using the texture shader\n";
            active_shader = texture_fragment_shader;
            texture_path = "spot_texture.png";
            r.set_texture(Texture(obj_path + texture_path));
        }
        else if (argc == 3 && std::string(argv[2]) == "normal")
        {
            std::cout << "Rasterizing using the normal shader\n";
            active_shader = normal_fragment_shader;
        }
        else if (argc == 3 && std::string(argv[2]) == "phong")
        {
            std::cout << "Rasterizing using the phong shader\n";
            active_shader = phong_fragment_shader;
        }
        else if (argc == 3 && std::string(argv[2]) == "bump")
        {
            std::cout << "Rasterizing using the bump shader\n";
            active_shader = bump_fragment_shader;
        }
        else if (argc == 3 && std::string(argv[2]) == "displacement")
        {
            std::cout << "Rasterizing using the displacement shader\n";
            active_shader = displacement_fragment_shader;
        }
    }

    Eigen::Vector3f eye_pos = {0, 0, 10};

    r.set_vertex_shader(vertex_shader);
    r.set_fragment_shader(active_shader);

    int key = 0;
    int frame_count = 0;

    if (command_line)
    {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);
        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45.0, 1, 0.1, 50));

        r.draw(TriangleList);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::cvtColor(image, image, cv::COLOR_RGB2BGR);

        cv::imwrite(filename, image);

        return 0;
    }

    while (key != 27)
    {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45.0, 1, 0.1, 50));

        //r.draw(pos_id, ind_id, col_id, rst::Primitive::Triangle);
        r.draw(TriangleList);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::cvtColor(image, image, cv::COLOR_RGB2BGR);

        cv::imshow("image", image);
        cv::imwrite(filename, image);
        key = cv::waitKey(10);

        if (key == 'a')
        {
            angle -= 0.1;
        }
        else if (key == 'd')
        {
            angle += 0.1;
        }
    }
    return 0;
}
