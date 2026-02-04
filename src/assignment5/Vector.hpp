#pragma once

#include <cmath>
#include <iostream>

class Vector3f
{
public:
    Vector3f()
        : x(0)
        , y(0)
        , z(0)
    {}
    Vector3f(float xx)
        : x(xx)
        , y(xx)
        , z(xx)
    {}
    Vector3f(float xx, float yy, float zz)
        : x(xx)
        , y(yy)
        , z(zz)
    {}
    // 重载乘法运算符,将向量与标量相乘
    Vector3f operator*(const float& r) const
    {
        return Vector3f(x * r, y * r, z * r);
    }
    // 重载除法运算符,将向量与标量相除
    Vector3f operator/(const float& r) const
    {
        return Vector3f(x / r, y / r, z / r);
    }

    // 重载乘法运算符,将向量与向量相乘(分量相乘)
    Vector3f operator*(const Vector3f& v) const
    {
        return Vector3f(x * v.x, y * v.y, z * v.z);
    }
    // 重载减法运算符,将向量与向量相减(分量相减)
    Vector3f operator-(const Vector3f& v) const
    {
        return Vector3f(x - v.x, y - v.y, z - v.z);
    }
    // 重载加法运算符,将向量与向量相加(分量相加)
    Vector3f operator+(const Vector3f& v) const
    {
        return Vector3f(x + v.x, y + v.y, z + v.z);
    }
    // 重载负号运算符,将向量取反
    Vector3f operator-() const
    {
        return Vector3f(-x, -y, -z);
    }
    // 重载加法赋值运算符,将向量与向量相加(分量相加)
    Vector3f& operator+=(const Vector3f& v)
    {
        x += v.x, y += v.y, z += v.z;
        // 这里的 *this 表示当前对象自身的引用（即对当前 Vector3f 实例的引用），
        // 用于支持链式赋值，例如 a += b += c;
        return *this;
    }
    // 重载乘法运算符,将标量与向量相乘(分量相乘)
    // 友元函数（friend）允许该外部函数访问类的私有和保护成员。
    // 这里定义了一个重载的乘法运算符，使得 float * Vector3f 支持这种写法。
    friend Vector3f operator*(const float& r, const Vector3f& v)
    {
        return Vector3f(v.x * r, v.y * r, v.z * r);
    }
    // 重载输出运算符,将向量输出为字符串
    friend std::ostream& operator<<(std::ostream& os, const Vector3f& v)
    {
        return os << v.x << ", " << v.y << ", " << v.z;
    }
    float x, y, z;
};

class Vector2f
{
public:
    Vector2f()
        : x(0)
        , y(0)
    {}
    Vector2f(float xx)
        : x(xx)
        , y(xx)
    {}
    Vector2f(float xx, float yy) : x(xx), y(yy) {}
    // 重载乘法运算符,将向量与标量相乘
    Vector2f operator*(const float &r) const { return Vector2f(x * r, y * r); }
    // 重载加法运算符,将向量与向量相加(分量相加)
    Vector2f operator+(const Vector2f& v) const
    {
        return Vector2f(x + v.x, y + v.y);
    }
    float x, y;
};

// 插值函数,将向量a和向量b按比例t进行插值
inline Vector3f lerp(const Vector3f& a, const Vector3f& b, const float& t)
{
    return a * (1 - t) + b * t;
}

// 这些函数没有放到类中，是作为全局的内联函数（inline function）实现的，
// 因此在任何引用了本头文件的地方，可以直接调用，比如 normalize(v)、dotProduct(a, b)、crossProduct(a, b)。
// 不需要通过类名或对象来调用，只要能够访问到 Vector3f 类型即可。
// 示例：
// Vector3f v1, v2;
// auto n = normalize(v1);
// float d = dotProduct(v1, v2);
// auto c = crossProduct(v1, v2);

inline Vector3f normalize(const Vector3f& v)
{
    float mag2 = v.x * v.x + v.y * v.y + v.z * v.z;
    if (mag2 > 0)
    {
        float invMag = 1 / sqrtf(mag2);
        return Vector3f(v.x * invMag, v.y * invMag, v.z * invMag);
    }

    return v;
}

// 点积函数,计算向量a和向量b的点积
inline float dotProduct(const Vector3f& a, const Vector3f& b)
{
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

// 叉积函数,计算向量a和向量b的叉积
inline Vector3f crossProduct(const Vector3f& a, const Vector3f& b)
{
    return Vector3f(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x);
}
