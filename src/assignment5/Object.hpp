#pragma once

#include "Vector.hpp"
#include "global.hpp"

class Object
{
public:
    Object()
        : materialType(DIFFUSE_AND_GLOSSY)
        , ior(1.3)
        , Kd(0.8)
        , Ks(0.2)
        , diffuseColor(0.2)
        , specularExponent(25)
    {}

    virtual ~Object() = default;

    // 在参数列表中出现 float& 但没有变量名的情况，表示该参数在函数声明中被声明为引用类型，但未给出变量名。
    // 这通常意味着该参数在这个类的实现中不会被使用（例如在基类虚函数中）。
    // 主要用途有两个：
    // 1. 保持与继承体系中同名函数的签名一致（比如派生类重载时参数不能少），但暂时用不到该参数。
    // 2. 编译器忽略未使用的参数警告，或说明该参数仅为接口兼容性存在。
    // 例如：virtual bool intersect(const Vector3f&, const Vector3f&, float&, uint32_t&, Vector2f&) const = 0;
    // 说明必须保有这些参数，但在具体某些派生类实现可能部分参数无需用到（可省略变量名）。

    // 这是一个纯虚函数声明，= 0 表示这个函数在基类中不提供实现，必须由派生类实现（抽象类）。
    // const 说明该成员函数不会修改成员变量，仅能读取成员数据。
    virtual bool intersect(const Vector3f&, const Vector3f&, float&, uint32_t&, Vector2f&) const = 0;

    virtual void getSurfaceProperties(const Vector3f&, const Vector3f&, const uint32_t&, const Vector2f&, Vector3f&,
                                      Vector2f&) const = 0;

    virtual Vector3f evalDiffuseColor(const Vector2f&) const
    {
        return diffuseColor;
    }

    // material properties
    MaterialType materialType;       // 材质类型（如漫反射、镜面、折射等）
    float ior;                       // 折射率（index of refraction），用于玻璃、水等透明材质
    float Kd, Ks;                    // Kd: 漫反射系数，Ks: 镜面反射系数
    Vector3f diffuseColor;           // 漫反射颜色
    float specularExponent;          // 高光指数（镜面反射的粗糙度/光泽度，越高越聚焦）
};
