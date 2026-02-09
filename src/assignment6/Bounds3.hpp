//
// Created by LEI XU on 5/16/19.
//

#ifndef RAYTRACING_BOUNDS3_H
#define RAYTRACING_BOUNDS3_H
#include "Ray.hpp"
#include "Vector.hpp"
#include <limits>
#include <array>

// 3D axis-aligned bounding box class
class Bounds3
{
public:
    Vector3f pMin, pMax; // 定义边界框的两个对角点

    // 默认构造函数，初始化为反向的无穷大边界
    Bounds3()
    {
        double minNum = std::numeric_limits<double>::lowest();
        double maxNum = std::numeric_limits<double>::max();
        pMax = Vector3f(minNum, minNum, minNum);
        pMin = Vector3f(maxNum, maxNum, maxNum);
    }

    // 用单一点构造边界框
    Bounds3(const Vector3f p) : pMin(p), pMax(p) {}

    // 用两个点构造边界框，自动计算最小和最大坐标
    Bounds3(const Vector3f p1, const Vector3f p2)
    {
        pMin = Vector3f(fmin(p1.x, p2.x), fmin(p1.y, p2.y), fmin(p1.z, p2.z));
        pMax = Vector3f(fmax(p1.x, p2.x), fmax(p1.y, p2.y), fmax(p1.z, p2.z));
    }

    // 获取边界框的对角线向量
    Vector3f Diagonal() const { return pMax - pMin; }

    // 获取边界框最长的轴(0=x, 1=y, 2=z)
    int maxExtent() const
    {
        Vector3f d = Diagonal();
        if (d.x > d.y && d.x > d.z)
            return 0;
        else if (d.y > d.z)
            return 1;
        else
            return 2;
    }

    // 计算边界框的表面积
    double SurfaceArea() const
    {
        Vector3f d = Diagonal();
        return 2 * (d.x * d.y + d.x * d.z + d.y * d.z);
    }

    // 获取边界框的中心点
    Vector3f Centroid() { return 0.5 * pMin + 0.5 * pMax; }

    // 计算两个边界框的交集
    Bounds3 Intersect(const Bounds3 &b)
    {
        return Bounds3(Vector3f(fmax(pMin.x, b.pMin.x), fmax(pMin.y, b.pMin.y),
                                fmax(pMin.z, b.pMin.z)),
                       Vector3f(fmin(pMax.x, b.pMax.x), fmin(pMax.y, b.pMax.y),
                                fmin(pMax.z, b.pMax.z)));
    }

    // 计算点相对于边界框的偏移量(归一化坐标)
    Vector3f Offset(const Vector3f &p) const
    {
        Vector3f o = p - pMin;
        if (pMax.x > pMin.x)
            o.x /= pMax.x - pMin.x;
        if (pMax.y > pMin.y)
            o.y /= pMax.y - pMin.y;
        if (pMax.z > pMin.z)
            o.z /= pMax.z - pMin.z;
        return o;
    }

    // 检查两个边界框是否重叠
    bool Overlaps(const Bounds3 &b1, const Bounds3 &b2)
    {
        bool x = (b1.pMax.x >= b2.pMin.x) && (b1.pMin.x <= b2.pMax.x);
        bool y = (b1.pMax.y >= b2.pMin.y) && (b1.pMin.y <= b2.pMax.y);
        bool z = (b1.pMax.z >= b2.pMin.z) && (b1.pMin.z <= b2.pMax.z);
        return (x && y && z);
    }

    // 检查点是否在边界框内
    bool Inside(const Vector3f &p, const Bounds3 &b)
    {
        return (p.x >= b.pMin.x && p.x <= b.pMax.x && p.y >= b.pMin.y &&
                p.y <= b.pMax.y && p.z >= b.pMin.z && p.z <= b.pMax.z);
    }

    // 重载[]操作符，i=0返回pMin，否则返回pMax
    inline const Vector3f &operator[](int i) const
    {
        return (i == 0) ? pMin : pMax;
    }

    // 检查光线是否与边界框相交
    inline bool IntersectP(const Ray &ray, const Vector3f &invDir,
                           const std::array<int, 3> &dirisNeg) const;
};

// 光线与边界框的相交测试(Slab方法)
inline bool Bounds3::IntersectP(const Ray &ray, const Vector3f &invDir,
                                const std::array<int, 3> &dirIsNeg) const
{
    // invDir: 光线方向的倒数(1.0/x, 1.0/y, 1.0/z)，使用乘法以加快计算
    // dirIsNeg: 光线方向标志[int(x>0), int(y>0), int(z>0)]，用于简化逻辑
    // TODO 测试光线与边界框是否相交

    // 计算光线与三个坐标平面的交点参数t值
    float t_Min_x = (pMin.x - ray.origin.x) * invDir.x;
    float t_Min_y = (pMin.y - ray.origin.y) * invDir.y;
    float t_Min_z = (pMin.z - ray.origin.z) * invDir.z;
    float t_Max_x = (pMax.x - ray.origin.x) * invDir.x;
    float t_Max_y = (pMax.y - ray.origin.y) * invDir.y;
    float t_Max_z = (pMax.z - ray.origin.z) * invDir.z;

    // 根据光线方向调整t的最小和最大值
    if (dirIsNeg[0])
    {
        float temp = t_Min_x;
        t_Min_x = t_Max_x;
        t_Max_x = temp;
    }

    if (dirIsNeg[1])
    {
        float temp = t_Min_y;
        t_Min_y = t_Max_y;
        t_Max_y = temp;
    }

    if (dirIsNeg[2])
    {
        float temp = t_Min_z;
        t_Min_z = t_Max_z;
        t_Max_z = temp;
    }

    // 计算光线进入和射出边界框的t值
    float tEnter = std::max(t_Min_x, std::max(t_Min_y, t_Min_z));
    float tExit = std::min(t_Max_x, std::min(t_Max_y, t_Max_z));

    // 若tExit >= 0且tEnter <= tExit，则光线与边界框相交
    if (tExit >= 0 && tEnter <= tExit)
        return true;

    return false;
}

// 合并两个边界框
inline Bounds3 Union(const Bounds3 &b1, const Bounds3 &b2)
{
    Bounds3 ret;
    ret.pMin = Vector3f::Min(b1.pMin, b2.pMin);
    ret.pMax = Vector3f::Max(b1.pMax, b2.pMax);
    return ret;
}

// 扩展边界框以包含给定的点
inline Bounds3 Union(const Bounds3 &b, const Vector3f &p)
{
    Bounds3 ret;
    ret.pMin = Vector3f::Min(b.pMin, p);
    ret.pMax = Vector3f::Max(b.pMax, p);
    return ret;
}

#endif // RAYTRACING_BOUNDS3_H
