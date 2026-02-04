#pragma once

#include "Object.hpp"
#include "Vector.hpp"

class Sphere : public Object
{
public:
    Sphere(const Vector3f& c, const float& r)
        : center(c)
        , radius(r)
        , radius2(r * r)
    {}

    // intersect 函数声明的含义是：
    // 该函数用于判断从 orig（射线起点）出发，沿
    // dir（射线方向）的射线是否与当前球体相交。 参数说明：
    // - orig: 射线的起点（原点），类型为 Vector3f。
    // - dir: 射线的方向，类型为 Vector3f。
    // - tnear: 交点距离（如果相交，会写入“最近”的交点与 orig 的距离）。
    // - uint32_t&: 用于存储三角形索引（对 Sphere 不使用，可忽略）。
    // - Vector2f&: 用于存储表面参数坐标（对 Sphere 不使用，可忽略）。
    // 返回值为 bool：如果有交点返回 true，否则返回 false。

    // “函数声明语法含义” 是指解释下面函数声明的语法和每个部分的含义。例如：
    // bool intersect(const Vector3f& orig, const Vector3f& dir, float& tnear, uint32_t&, Vector2f&) const override;
    // 其中:
    // - bool: 表示函数的返回类型（这里返回值为布尔类型true/false）。
    // - intersect: 是函数名。
    // - ( ... ): 括号里的内容是参数列表，每个参数都指定了类型和用途。例如
    //   - const Vector3f& orig: 射线的起点，引用类型，且不会被修改。
    //   - const Vector3f& dir: 射线的方向，引用类型，且不会被修改。
    //   - float& tnear: 交点距离，作为输出参数用于写出最近交点距离。
    //   - uint32_t&: 用于传递三角形索引（在Sphere中可忽略）。
    //   - Vector2f&: 用于存储表面UV参数（在Sphere中可忽略）。
    // - const: 说明该成员函数不会修改该类的成员变量（保证只读）。
    // - override: 表示该函数是虚函数，重写了基类Object的同名虚函数。
    bool intersect(const Vector3f& orig, const Vector3f& dir, float& tnear, uint32_t&, Vector2f&) const override
    {
        // analytic solution
        Vector3f L = orig - center; // 向量L = 射线起点 - 球心
        float a = dotProduct(dir, dir); // 向量dir的点积
        float b = 2 * dotProduct(dir, L); // 向量dir和向量L的点积
        float c = dotProduct(L, L) - radius2; // 向量L的点积 - 半径的平方
        float t0, t1;
        if (!solveQuadratic(a, b, c, t0, t1)) // 解二次方程
            return false; // 如果解不存在，返回false
        if (t0 < 0)
            t0 = t1; // 如果t0小于0，则t0等于t1
        if (t0 < 0) // 如果t0小于0，返回false
            return false;
        tnear = t0;

        return true;
    }

    // 获取表面属性
    // 参数说明：
    // - const Vector3f& P: 交点位置，类型为 Vector3f。
    // - const Vector3f&: 不使用，可忽略。
    // - const uint32_t&: 不使用，可忽略。
    // - const Vector2f&: 不使用，可忽略。
    // - Vector3f& N: 法线，类型为 Vector3f。
    // - Vector2f&: 不使用，可忽略。
    // 返回值为 void。
    void getSurfaceProperties(const Vector3f& P, const Vector3f&, const uint32_t&, const Vector2f&,
                              Vector3f& N, Vector2f&) const override
    {
        N = normalize(P - center);
    }

    Vector3f center;
    float radius, radius2;
};
