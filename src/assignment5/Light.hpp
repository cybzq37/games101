#pragma once

#include "Vector.hpp"

class Light
{
public:
    // 这种语法是C++中的构造函数初始化列表（Constructor Initialization List）。
    // 它的意思是在构造函数体执行之前，用传入的参数（p, i）分别初始化成员变量 position 和 intensity。
    // 这样写的效果类似于：
    // Light(const Vector3f& p, const Vector3f& i) {
    //     position = p;
    //     intensity = i;
    // }
    // 但初始化列表语法通常效率更高，尤其是对于类类型成员变量。
    Light(const Vector3f& p, const Vector3f& i)
        : position(p), intensity(i)
    {}

    // = default 的含义：表示编译器生成默认的析构函数实现。
    virtual ~Light() = default;
    Vector3f position;
    Vector3f intensity;
};
