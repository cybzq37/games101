#include "rasterizer.h"
#include "Eigen/src/Core/Matrix.h"

#include <algorithm>
#include <opencv2/opencv.hpp>
#include <Eigen/Eigen>
#include <math.h>
#include <stdexcept>

using namespace Eigen;
using namespace std;

/**
 * 加载顶点位置数据到缓冲区
 * @param positions 顶点位置数组，每个元素是一个 3D 坐标 (x, y, z)
 * @return 返回缓冲区 ID，用于后续绘制时引用这些顶点数据
 */
rst::pos_buf_id rst::Rasterizer::load_positions(const std::vector<Eigen::Vector3f> &positions)
{
    // 获取一个新的唯一缓冲区 ID
    auto id = get_next_id();

    // 将顶点位置数据存储到 pos_buf 映射中
    // emplace 直接在 map 中构造键值对，避免不必要的拷贝
    pos_buf.emplace(id, positions);

    // 返回包含缓冲区 ID 的结构体
    return {id};
}

/**
 * 加载顶点索引数据到缓冲区
 * @param indices 顶点索引数组，每个元素是三个整数，表示一个三角形的三个顶点索引
 *                例如：{0, 1, 2} 表示使用 positions[0], positions[1], positions[2] 构成三角形
 * @return 返回缓冲区 ID，用于后续绘制时引用这些索引数据
 */
rst::ind_buf_id rst::Rasterizer::load_indices(const std::vector<Eigen::Vector3i> &indices)
{
    // 获取一个新的唯一缓冲区 ID
    auto id = get_next_id();

    // 将顶点索引数据存储到 ind_buf 映射中
    // emplace 直接在 map 中构造键值对，避免不必要的拷贝
    ind_buf.emplace(id, indices);

    // 返回包含缓冲区 ID 的结构体
    return {id};
}

auto to_vec4(const Eigen::Vector3f& v3, float w = 1.0f) {
    return Eigen::Vector4f(v3(0), v3(1), v3(2), w);
}

/**
 * Bresenham 线段绘制算法
 * 使用整数运算高效地绘制线段，避免浮点运算和除法
 * 参考：https://stackoverflow.com/a/16405254
 *
 * @param begin 线段起点（3D 坐标）
 * @param end 线段终点（3D 坐标）
 */
void rst::Rasterizer::draw_line(Eigen::Vector3f begin, Eigen::Vector3f end) {
  // 提取起点和终点的坐标
  auto x1 = begin.x();
  auto y1 = begin.y();
  auto x2 = end.x();
  auto y2 = end.y();

  // 设置线段颜色为白色（RGB: 255, 255, 255）
  Eigen::Vector3f line_color = {255, 255, 255};

  // 算法变量声明
  int x, y;        // 当前像素坐标
  int dx, dy;      // 坐标差值（有符号）
  int dx1, dy1;    // 坐标差值的绝对值
  int px, py;      // 决策参数（用于决定下一个像素位置）
  int xe, ye;      // 终点坐标（根据方向确定）
  int i;           // 循环计数器

  // 计算坐标差值
  dx = x2 - x1;    // x 方向差值
  dy = y2 - y1;    // y 方向差值
  dx1 = fabs(dx);  // x 方向差值的绝对值
  dy1 = fabs(dy);  // y 方向差值的绝对值

  // 初始化决策参数（Bresenham 算法的核心）
  // px: 当 |dx| >= |dy| 时使用的决策参数
  px = 2 * dy1 - dx1;
  // py: 当 |dy| > |dx| 时使用的决策参数
  py = 2 * dx1 - dy1;

  // 情况 1：线段在 x 方向变化更大（|dy| <= |dx|）
  // 此时沿 x 轴逐像素绘制，y 坐标根据决策参数调整
  if(dy1 <= dx1)
  {
      // 确定绘制方向：从左到右（dx >= 0）或从右到左（dx < 0）
      if(dx >= 0)
      {
          // 从左到右绘制
          x = x1;
          y = y1;
          xe = x2;  // 终点 x 坐标
      }
      else
      {
          // 从右到左绘制（交换起点和终点）
          x = x2;
          y = y2;
          xe = x1;  // 终点 x 坐标
      }

      // 绘制起点
      Eigen::Vector3f point = Eigen::Vector3f(x, y, 1.0f);
      set_pixel(point, line_color);

      // 沿 x 轴逐像素绘制
      for(i = 0; x < xe; i++)
      {
          x = x + 1;  // x 坐标每次递增 1

          // 根据决策参数决定 y 坐标是否变化
          if(px < 0)
          {
              // 决策参数 < 0：y 坐标不变，只更新决策参数
              px = px + 2 * dy1;
          }
          else
          {
              // 决策参数 >= 0：y 坐标需要变化
              // 判断 y 的变化方向
              if((dx < 0 && dy < 0) || (dx > 0 && dy > 0))
              {
                  // 斜率为正：y 递增
                  y = y + 1;
              }
              else
              {
                  // 斜率为负：y 递减
                  y = y - 1;
              }
              // 更新决策参数
              px = px + 2 * (dy1 - dx1);
          }

          // 绘制当前像素
          Eigen::Vector3f point = Eigen::Vector3f(x, y, 1.0f);
          set_pixel(point, line_color);
      }
  }
  // 情况 2：线段在 y 方向变化更大（|dy| > |dx|）
  // 此时沿 y 轴逐像素绘制，x 坐标根据决策参数调整
  else
  {
      // 确定绘制方向：从上到下（dy >= 0）或从下到上（dy < 0）
      if(dy >= 0)
      {
          // 从上到下绘制
          x = x1;
          y = y1;
          ye = y2;  // 终点 y 坐标
      }
      else
      {
          // 从下到上绘制（交换起点和终点）
          x = x2;
          y = y2;
          ye = y1;  // 终点 y 坐标
      }

      // 绘制起点
      Eigen::Vector3f point = Eigen::Vector3f(x, y, 1.0f);
      set_pixel(point, line_color);

      // 沿 y 轴逐像素绘制
      for(i = 0; y < ye; i++)
      {
          y = y + 1;  // y 坐标每次递增 1

          // 根据决策参数决定 x 坐标是否变化
          if(py <= 0)
          {
              // 决策参数 <= 0：x 坐标不变，只更新决策参数
              py = py + 2 * dx1;
          }
          else
          {
              // 决策参数 > 0：x 坐标需要变化
              // 判断 x 的变化方向
              if((dx < 0 && dy < 0) || (dx > 0 && dy > 0))
              {
                  // 斜率为正：x 递增
                  x = x + 1;
              }
              else
              {
                  // 斜率为负：x 递减
                  x = x - 1;
              }
              // 更新决策参数
              py = py + 2 * (dx1 - dy1);
          }

          // 绘制当前像素
          Eigen::Vector3f point = Eigen::Vector3f(x, y, 1.0f);
          set_pixel(point, line_color);
      }
  }
}

/**
 * 绘制图元（图形渲染管线的主函数）
 * 执行完整的图形渲染流程：模型变换 → 视图变换 → 投影变换 → 透视除法 → 视口变换 → 光栅化
 *
 * @param pos_buffer 顶点位置缓冲区 ID
 * @param ind_buffer 顶点索引缓冲区 ID
 * @param type 图元类型（目前仅支持三角形）
 */
void rst::Rasterizer::draw(rst::pos_buf_id pos_buffer,
                           rst::ind_buf_id ind_buffer, rst::Primitive type)
{
    // 检查图元类型：目前仅支持三角形绘制
    if (type != rst::Primitive::Triangle)
    {
        throw std::runtime_error("Drawing primitives other than triangle is not implemented yet!");
    }

    // 获取顶点位置缓冲区和索引缓冲区的引用
    auto& buf = pos_buf[pos_buffer.pos_id];  // 顶点位置数组
    auto& ind = ind_buf[ind_buffer.ind_id];  // 顶点索引数组（每个元素是三个顶点的索引）

    // 深度值线性变换参数
    // 将 NDC 坐标系的 z 值（范围 [-1, 1]）映射到深度缓冲区范围 [0.1, 100]
    // f1: 缩放因子 = (far - near) / 2 = (100 - 0.1) / 2
    float f1 = (100 - 0.1) / 2.0;
    // f2: 偏移量 = (far + near) / 2 = (100 + 0.1) / 2
    float f2 = (100 + 0.1) / 2.0;

    // 计算 MVP 矩阵（Model-View-Projection）
    // 矩阵乘法顺序：projection * view * model
    // 注意：矩阵乘法从右到左应用变换
    // 变换顺序：模型空间 → 世界空间 → 相机空间 → 裁剪空间
    Eigen::Matrix4f mvp = projection * view * model;

    // 遍历每个三角形（每个索引组定义一个三角形）
    for (auto& i : ind)
    {
        Triangle t;  // 创建三角形对象

        // ==================== 步骤 1: MVP 变换 ====================
        // 对三角形的三个顶点进行 MVP 变换
        // 将顶点从模型空间变换到裁剪空间（齐次坐标）
        Eigen::Vector4f v[] = {
                mvp * to_vec4(buf[i[0]], 1.0f),  // 顶点 0：转换为 4D 齐次坐标后应用 MVP 变换
                mvp * to_vec4(buf[i[1]], 1.0f),  // 顶点 1：转换为 4D 齐次坐标后应用 MVP 变换
                mvp * to_vec4(buf[i[2]], 1.0f)   // 顶点 2：转换为 4D 齐次坐标后应用 MVP 变换
        };

        // ==================== 步骤 2: 透视除法 ====================
        // 将齐次坐标转换为 NDC（标准化设备坐标）
        // 除以 w 分量：将裁剪空间坐标转换为 NDC 坐标（范围 [-1, 1]）
        for (auto& vec : v) {
            vec /= vec.w();  // 透视除法：x/w, y/w, z/w, w/w=1
        }

        // ==================== 步骤 3: 视口变换 ====================
        // 将 NDC 坐标转换为屏幕坐标（像素坐标）
        for (auto & vert : v)
        {
            // x 坐标变换：从 [-1, 1] 映射到 [0, width]
            // 公式：screen_x = 0.5 * width * (ndc_x + 1.0)
            vert.x() = 0.5 * width * (vert.x() + 1.0);

            // y 坐标变换：从 [-1, 1] 映射到 [0, height]
            // 公式：screen_y = 0.5 * height * (ndc_y + 1.0)
            vert.y() = 0.5 * height * (vert.y() + 1.0);

            // z 坐标变换：从 [-1, 1] 映射到深度缓冲区范围 [0.1, 100]
            // 公式：depth = ndc_z * f1 + f2
            // 线性映射：ndc_z ∈ [-1, 1] → depth ∈ [0.1, 100]
            vert.z() = vert.z() * f1 + f2;
        }

        // ==================== 步骤 4: 设置三角形顶点 ====================
        // 将变换后的顶点坐标设置到三角形对象中
        // head<3>() 提取前三个分量（x, y, z），丢弃 w 分量
        for (int i = 0; i < 3; ++i)
        {
            t.setVertex(i, v[i].head<3>());  // 设置第 i 个顶点的坐标
        }

        // ==================== 步骤 5: 设置顶点颜色 ====================
        // 为每个顶点设置不同的颜色（用于调试和可视化）
        t.setColor(0, 255.0,  0.0,  0.0);  // 顶点 0：红色
        t.setColor(1, 0.0  ,255.0,  0.0);  // 顶点 1：绿色
        t.setColor(2, 0.0  ,  0.0,255.0);  // 顶点 2：蓝色

        // ==================== 步骤 6: 光栅化 ====================
        // 将三角形光栅化为线框（绘制三条边）
        rasterize_wireframe(t);
    }
}

void rst::Rasterizer::rasterize_wireframe(const Triangle &t) {
  draw_line(t.c(), t.a());
  draw_line(t.a(), t.b());
  draw_line(t.b(), t.c());
}

void rst::Rasterizer::set_model(const Eigen::Matrix4f& m)
{
    model = m;
}

void rst::Rasterizer::set_view(const Eigen::Matrix4f& v)
{
    view = v;
}

void rst::Rasterizer::set_projection(const Eigen::Matrix4f& p)
{
    projection = p;
}

void rst::Rasterizer::clear(rst::Buffers buff) {
  if((buff & rst::Buffers::Color) == rst::Buffers::Color) { // 位运算，如果buff包含颜色缓冲区
    std::fill(frame_buf.begin(), frame_buf.end(), Eigen::Vector3f(0, 0, 0));
  }
  if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth) { // 位运算，如果buff包含深度缓冲区
    std::fill(depth_buf.begin(), depth_buf.end(), std::numeric_limits<float>::infinity());
  }
}

/**
 * 光栅化器构造函数
 * @param w 画布宽度（像素）
 * @param h 画布高度（像素）
 */
rst::Rasterizer::Rasterizer(int w, int h) : width(w), height(h) {
  // 初始化列表：使用成员初始化列表初始化 width 和 height
  // 这种方式比在函数体内赋值更高效，因为直接构造而不是先默认构造再赋值

  // 调整帧缓冲区大小：为每个像素分配颜色存储空间
  // 总像素数 = 宽度 × 高度，每个像素存储一个 Vector3f (RGB 颜色)
  frame_buf.resize(w * h);

  // 调整深度缓冲区大小：为每个像素分配深度值存储空间
  // 总像素数 = 宽度 × 高度，每个像素存储一个 float (深度值，用于深度测试)
  depth_buf.resize(w * h);
}

/**
 * 获取像素在缓冲区中的索引
 * @param x 像素 x 坐标
 * @param y 像素 y 坐标
 * @return 在缓冲区中的一维索引
 */
int rst::Rasterizer::get_index(int x, int y) const {
  return (height - y) * width + x;
}

/**
 * 设置指定像素的颜色
 * @param point 像素位置（3D 坐标，x 和 y 为屏幕坐标，z 用于深度测试）
 * @param color 像素颜色（RGB，范围 0.0-1.0）
 */
void rst::Rasterizer::set_pixel(const Eigen::Vector3f& point, const Eigen::Vector3f& color) {
    // 边界检查：确保像素坐标在有效范围内
    // 如果坐标超出画布范围，直接返回（不绘制）
    if (point.x() < 0 || point.x() >= width ||
        point.y() < 0 || point.y() >= height) return;

    // 将 2D 屏幕坐标 (x, y) 转换为 1D 缓冲区索引
    // 公式：index = (height - y) * width + x
    // 注意：y 坐标需要翻转，因为屏幕坐标系（y 向下）与缓冲区存储顺序（y 向上）相反
    // 旧公式（已废弃）：auto ind = point.y() + point.x() * width;
    auto ind = (height - point.y()) * width + point.x();

    // 将颜色写入帧缓冲区
    frame_buf[ind] = color;
}
