#pragma once
#include "Scene.hpp"

// 定义一个结构体，用于存储射线与物体相交的信息
struct hit_payload
{
    float tNear; // 射线与物体相交的距离
    uint32_t index; // 物体索引
    Vector2f uv; // 物体纹理坐标
    Object* hit_obj; // 射线与物体相交的物体
};

class Renderer
{
public:
    void Render(const Scene& scene);

private:
};
