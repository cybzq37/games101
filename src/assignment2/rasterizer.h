#pragma once

#include "Triangle.h"
#include <algorithm>
#include <Eigen/Eigen>

using namespace Eigen;

namespace rst {

enum class Buffers {
  Color = 1,
  Depth = 2,
};

// 枚举类默认不支持位运算，需要重载运算符
inline Buffers operator|(Buffers a, Buffers b) {
    return Buffers(int(a) | int(b));
}

inline Buffers operator&(Buffers a, Buffers b) {
    return Buffers(int(a) & int(b));
}

enum class Primitive {
  Line,
  Triangle,
};

/*
 * 说明：draw 函数接受多个缓冲区 ID 作为参数。
 * 这些结构体确保如果你搞混了它们的顺序，编译器将无法编译通过。
 * 这就是类型安全（Type safety）
 * */
struct pos_buf_id {
    int pos_id = 0;
};
struct ind_buf_id {
    int ind_id = 0;
};

struct col_buf_id {
    int col_id = 0;
};

/**
 * 光栅化器类
 * 负责将 3D 图形转换为 2D 像素图像
 */
class Rasterizer {
public:
  /**
   * 构造函数
   * @param w 画布宽度（像素）
   * @param h 画布高度（像素）
   */
  Rasterizer(int w, int h);

  /**
   * 加载顶点位置数据
   * @param positions 顶点位置数组（3D 坐标）
   * @return 返回缓冲区 ID，用于后续绘制
   */
  pos_buf_id load_positions(const std::vector<Eigen::Vector3f>& positions);

  /**
   * 加载顶点索引数据
   * @param indices 顶点索引数组（用于定义三角形或线段）
   * @return 返回缓冲区 ID，用于后续绘制
   */
  ind_buf_id load_indices(const std::vector<Eigen::Vector3i>& indices);

  /**
   * 加载顶点颜色数据
   * @param colors 顶点颜色数组（RGB 颜色）
   * @return 返回缓冲区 ID，用于后续绘制
   */
  col_buf_id load_colors(const std::vector<Eigen::Vector3f> &colors);

  /**
   * 设置模型变换矩阵
   * @param m 4x4 模型变换矩阵（从模型空间到世界空间）
   */
  void set_model(const Eigen::Matrix4f& m);

  /**
   * 设置视图变换矩阵
   * @param v 4x4 视图变换矩阵（从世界空间到相机空间）
   */
  void set_view(const Eigen::Matrix4f& v);

  /**
   * 设置投影变换矩阵
   * @param p 4x4 投影变换矩阵（从相机空间到裁剪空间）
   */
  void set_projection(const Eigen::Matrix4f& p);

  /**
   * 设置像素颜色
   * @param point 像素位置（3D 坐标，z 用于深度测试）
   * @param color 像素颜色（RGB，范围 0.0-1.0）
   */
  void set_pixel(const Eigen::Vector3f& point, const Eigen::Vector3f& color);

  /**
   * 清空缓冲区
   * @param buff 要清空的缓冲区类型（颜色缓冲区、深度缓冲区或两者）
   */
  void clear(Buffers buff);

  /**
   * 绘制图元
   * @param pos_buffer 顶点位置缓冲区 ID
   * @param ind_buffer 顶点索引缓冲区 ID
   * @param type 图元类型（线段或三角形）
   */
  void draw(pos_buf_id pos_buffer, ind_buf_id ind_buffer, col_buf_id col_buffer, Primitive type);

  /**
   * 获取帧缓冲区（用于读取渲染结果）
   * @return 帧缓冲区的引用，包含所有像素的颜色信息
   */
  std::vector<Eigen::Vector3f> &frame_buffer() { return frame_buf; }

private:
  // ==================== 变换矩阵 ====================
  Eigen::Matrix4f model;      // 模型变换矩阵（模型空间 → 世界空间）
  Eigen::Matrix4f view;       // 视图变换矩阵（世界空间 → 相机空间）
  Eigen::Matrix4f projection; // 投影变换矩阵（相机空间 → 裁剪空间）

  // ==================== 缓冲区 ====================
  std::map<int, std::vector<Eigen::Vector3f>> pos_buf; // 顶点位置缓冲区（ID → 顶点数组）
  std::map<int, std::vector<Eigen::Vector3i>> ind_buf; // 顶点索引缓冲区（ID → 索引数组，i表示整数）
  std::map<int, std::vector<Eigen::Vector3f>> col_buf; // 顶点颜色缓冲区（ID → 颜色数组）

  std::vector<Eigen::Vector3f> frame_buf; // 帧缓冲区（存储每个像素的 RGB 颜色）
  std::vector<float> depth_buf;           // 深度缓冲区（存储每个像素的深度值，用于深度测试）

  // 超采样抗锯齿（2x2 SSAA，每个像素有4个子采样点）
  std::vector<std::vector<Eigen::Vector3f>> frame_buf_2xSSAA;
  std::vector<std::vector<float>> depth_buf_2xSSAA;

  /**
   * 获取像素在缓冲区中的索引（内部辅助函数）
   * @param x 像素 x 坐标
   * @param y 像素 y 坐标
   * @return 在缓冲区中的一维索引
   */
  int get_index(int x, int y) const;

  // ==================== 画布尺寸 ====================
  int width, height;  // 画布宽度和高度（像素）

  // ==================== ID 管理 ====================
  int next_id = 0;  // 下一个可用的缓冲区 ID

  /**
   * 获取下一个缓冲区 ID（内部辅助函数）
   * @return 新的缓冲区 ID
   */
  int get_next_id() { return next_id++; }


    /**
   * 绘制线段（内部辅助函数）
   * @param begin 线段起点（3D 坐标）
   * @param end 线段终点（3D 坐标）
   */
   void draw_line(Eigen::Vector3f begin, Eigen::Vector3f end);

   /**
    * 光栅化线框三角形（内部辅助函数）
    * @param t 要绘制的三角形对象
    */
   void rasterize_wireframe(const Triangle &t);

};
}
