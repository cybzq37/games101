#include "rasterizer.h"
#include "Eigen/src/Core/Matrix.h"

#include <algorithm>
#include <array>
#include <opencv2/opencv.hpp>
#include <Eigen/Eigen>
#include <math.h>
#include <stdexcept>

using namespace Eigen;
using namespace std;

#define SSAA false

/**
 * 加载顶点位置数据到缓冲区
 * @param positions 顶点位置数组，每个元素是一个 3D 坐标 (x, y, z)
 * @return 返回缓冲区 ID，用于后续绘制时引用这些顶点数据
 */
rst::pos_buf_id rst::Rasterizer::load_positions(const std::vector<Eigen::Vector3f> &positions)
{
    auto id = get_next_id();  // 获取一个新的唯一缓冲区 ID
    pos_buf.emplace(id, positions); // 将顶点位置数据存储到 pos_buf 映射中
	return { id }; // 返回包含缓冲区 ID 的结构体
}

/**
 * 加载顶点索引数据到缓冲区
 * @param indices 顶点索引数组，每个元素是三个整数，表示一个三角形的三个顶点索引
 *                例如：{0, 1, 2} 表示使用 positions[0], positions[1], positions[2] 构成三角形
 * @return 返回缓冲区 ID，用于后续绘制时引用这些索引数据
 */
rst::ind_buf_id rst::Rasterizer::load_indices(const std::vector<Eigen::Vector3i> &indices)
{
    auto id = get_next_id();
    ind_buf.emplace(id, indices);
    return {id};
}

rst::col_buf_id rst::Rasterizer::load_colors(const std::vector<Eigen::Vector3f> &colors)
{
    auto id = get_next_id();
    col_buf.emplace(id, colors);
    return {id};
}

auto to_vec4(const Eigen::Vector3f& v3, float w = 1.0f) {
    return Eigen::Vector4f(v3(0), v3(1), v3(2), w);
}

/**
 * 判断点是否在三角形内（使用叉积法）
 *
 * 算法原理：
 * 1. 计算三角形三条边的向量：ab, bc, ca
 * 2. 计算测试点到三个顶点的向量：ap, bp, cp
 * 3. 计算叉积：ab×ap, bc×bp, ca×cp
 * 4. 如果所有叉积的z分量同号（都>0或都<0），则点在三角形内部
 *
 * 注意：
 * - 假设三角形顶点按逆时针顺序排列（根据Triangle.h注释）
 * - 点在边上时（叉积为0）会被判定为外部，这是光栅化的常见做法
 *
 * @param x 点 x 坐标
 * @param y 点 y 坐标
 * @param v 三角形顶点数组（3个Vector3f，按逆时针顺序）
 * @return 是否在三角形内
 */
static bool insideTriangle(int x, int y, const std::array<Vector4f, 3>& v) {
	 // 计算三条边的向量（从v0到v1，v1到v2，v2到v0）
	 // 注意：v 是 Vector4f 数组，需要提取前三个分量（x, y, z）用于 2D 判断
	 Eigen::Vector3f ab = (v[1] - v[0]).head<3>();
	 Eigen::Vector3f bc = (v[2] - v[1]).head<3>();
	 Eigen::Vector3f ca = (v[0] - v[2]).head<3>();

	 // 计算测试点到三个顶点的向量
	 Eigen::Vector3f ap(x - v[0].x(), y - v[0].y(), 0);
	 Eigen::Vector3f bp(x - v[1].x(), y - v[1].y(), 0);
	 Eigen::Vector3f cp(x - v[2].x(), y - v[2].y(), 0);

	 // 计算叉积（只关心z分量，因为所有向量都在xy平面）
	 float z1 = ab.cross(ap).z();
	 float z2 = bc.cross(bp).z();
	 float z3 = ca.cross(cp).z();

	 // 如果所有叉积z分量同号（都>0或都<0），则点在三角形内
	 // 注意：点在边上时（z=0）会被判定为外部
	 return (z1 > 0 && z2 > 0 && z3 > 0) || (z1 < 0 && z2 < 0 && z3 < 0);
}

 void rst::Rasterizer::draw(rst::pos_buf_id pos_buffer, rst::ind_buf_id ind_buffer, rst::col_buf_id col_buffer,
           Primitive type) {
	 auto& buf = pos_buf[pos_buffer.pos_id];
	 auto& ind = ind_buf[ind_buffer.ind_id];
	 auto& col = col_buf[col_buffer.col_id];

     // f1 和 f2 用于将NDC的z值（范围[-1,1]）线性映射到深度缓冲区的实际深度范围[near, far]（这里是0.1到50）
     // 转换公式: z_buffer = z_ndc * f1 + f2
     // 这里的50来源于远裁剪面的z值zFar（assignment2.cpp中get_projection_matrix调用里传入的参数）
     // 画面的有效深度区间是[0.1, 50]，也即近平面为0.1，远平面为50
     float f1 = (50 - 0.1) / 2.0; // 一半的深度范围宽度
     float f2 = (50 + 0.1) / 2.0; // 深度范围的中心值

	 Eigen::Matrix4f mvp = projection * view * model;
	 for (auto& i : ind) // 处理每个顶点索引三元组
     {
         Triangle t;
		 Eigen::Vector4f v[] = { // 对3个顶点进行MVP变换
             mvp * to_vec4(buf[i[0]], 1.0f),
             mvp * to_vec4(buf[i[1]], 1.0f),
             mvp * to_vec4(buf[i[2]], 1.0f)
         };
         // Homogeneous division  透视除法.转为NDC坐标
         for (auto& vec : v) {
             vec /= vec.w();
         }
		 // 转为屏幕坐标
         for (auto& vert : v)
         {
             vert.x() = 0.5 * width * (vert.x() + 1.0); // x映射到0到width
             vert.y() = 0.5 * height * (vert.y() + 1.0); // y映射到0到height
             vert.z() = vert.z() * f1 + f2; // z映射到深度缓冲的 [near, far], 取值为0.1到50
         }

         for (int i = 0; i < 3; ++i)
         {
             t.setVertex(i, v[i].head<3>()); // 设置三角形第i个顶点的坐标
         }


         auto col_x = col[i[0]];
         auto col_y = col[i[1]];
         auto col_z = col[i[2]];

         t.setColor(0, col_x[0], col_x[1], col_x[2]);
         t.setColor(1, col_y[0], col_y[1], col_y[2]);
         t.setColor(2, col_z[0], col_z[1], col_z[2]);

         rasterize_wireframe(t);
     }
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
    if ((buff & rst::Buffers::Color) == rst::Buffers::Color) // 位运算，如果buff包含颜色缓冲区
    {
        std::fill(frame_buf.begin(), frame_buf.end(), Eigen::Vector3f{0, 0, 0});
        //SSAA Begin
        for (int i = 0; i < frame_buf_2xSSAA.size(); i++) {
            frame_buf_2xSSAA[i].resize(4);  // 每个像素有4个子采样点（2x2 SSAA）
            std::fill(frame_buf_2xSSAA[i].begin(), frame_buf_2xSSAA[i].end(), Eigen::Vector3f{0, 0, 0});
        }
        //SSAA End
    }
    if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth) // 位运算，如果buff包含深度缓冲区
    {
        std::fill(depth_buf.begin(), depth_buf.end(), std::numeric_limits<float>::infinity());
        //SSAA Begin
        for (int i = 0; i < depth_buf_2xSSAA.size(); i++) {
            depth_buf_2xSSAA[i].resize(4);
            std::fill(depth_buf_2xSSAA[i].begin(), depth_buf_2xSSAA[i].end(), std::numeric_limits<float>::infinity());
        }
        //SSAA End
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

  // 超采样抗锯齿
  frame_buf_2xSSAA.resize(w * h);
  depth_buf_2xSSAA.resize(w * h);
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


// computeBarycentric2D 是一个仅在当前编译单元（当前 cpp 文件）可见的静态函数
// “static”关键字限定了其作用域仅为本源文件（编译单元）内部
// 这种文件作用域的静态方法通常放在全局命名空间、类/命名空间定义之外
// 只要在使用前定义即可，不要求放在文件开头或结尾，但通常会放在相关实现附近，方便逻辑归类
static std::tuple<float, float, float> computeBarycentric2D(float x, float y, const Vector3f* v)
{
    float c1 = (x*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*y + v[1].x()*v[2].y() - v[2].x()*v[1].y()) / (v[0].x()*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*v[0].y() + v[1].x()*v[2].y() - v[2].x()*v[1].y());
    float c2 = (x*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*y + v[2].x()*v[0].y() - v[0].x()*v[2].y()) / (v[1].x()*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*v[1].y() + v[2].x()*v[0].y() - v[0].x()*v[2].y());
    float c3 = (x*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*y + v[0].x()*v[1].y() - v[1].x()*v[0].y()) / (v[2].x()*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*v[2].y() + v[0].x()*v[1].y() - v[1].x()*v[0].y());
    return {c1,c2,c3};
}

/**
 * 绘制图元（图形渲染管线的主函数）
 * 执行完整的图形渲染流程：MVP变换 → 透视除法 → 光栅化
 *
 * @param pos_buffer 顶点位置缓冲区 ID
 * @param ind_buffer 顶点索引缓冲区 ID
 * @param type 图元类型（目前仅支持三角形）
 */
 void rst::Rasterizer::rasterize_wireframe(const Triangle &t) {
    auto v = t.toVector4();

    // TODO : Find out the bounding box of current triangle.
    // iterate through the pixel and find if the current pixel is inside the
    // triangle
    float min_x = std::min(v[0][0], std::min(v[1][0], v[2][0]));
    float max_x = std::max(v[0][0], std::max(v[1][0], v[2][0]));
    float min_y = std::min(v[0][1], std::min(v[1][1], v[2][1]));
    float max_y = std::max(v[0][1], std::max(v[1][1], v[2][1]));

    for (int x = min_x; x < max_x; x++) {
      for (int y = min_y; y < max_y; y++) {
        if (insideTriangle(x, y, v)) { // 如果当前像素在三角形内
          float min_depth = FLT_MAX;
          if (SSAA) {
            int index = 0;
            for (float i = 0.25; i < 1.0; i += 0.5) {
              for (float j = 0.25; j < 1.0; j += 0.5) {
                // x2SSAA坐标
				  auto [apha, beta, gamma] = computeBarycentric2D(x + i, y + j, t.v); // xy是屏幕坐标, t是深度, a,b,g是重力坐标
                  float z_interpolated =
                      apha * v[0].z() / v[0].w() +
                      beta * v[1].z() / v[1].w() +
                      gamma * v[2].z() / v[2].w();
                  float w_reciprocal = 1.0 / (apha / v[0].w() + beta / v[0].w() + gamma / v[2].w());
                  z_interpolated *= w_reciprocal;  // 对屏幕坐标进行插值校正
                  min_depth = std::min(min_depth, z_interpolated);
                  if (min_depth < depth_buf_2xSSAA[get_index(x, y)][index]) {
                      frame_buf_2xSSAA[get_index(x, y)][index] = t.getColor();
                      depth_buf_2xSSAA[get_index(x, y)][index] = min_depth;
                  }
              }
              index++;
            }
          } else {
              auto [alpha, beta, gamma] = computeBarycentric2D(x, y, t.v);
              float w_reciprocal = 1.0 / (alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
              float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
              z_interpolated *= w_reciprocal;
              min_depth = std::min(min_depth, z_interpolated);
              if (min_depth < depth_buf[get_index(x, y)])
              {
                  depth_buf[get_index(x, y)] = min_depth;
                  Eigen::Vector3f point(x, y, 1.0f);
                  set_pixel(point, t.getColor());
              }
          }

        }
      }
    }
   }
