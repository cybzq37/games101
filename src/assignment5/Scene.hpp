#pragma once

#include <vector>
#include <memory>
#include "Vector.hpp"
#include "Object.hpp"
#include "Light.hpp"

class Scene
{
public:
    // setting up options: 设置场景的宽度、高度、FOV、背景颜色、最大深度、epsilon
    int width = 1280;
    int height = 960;
    double fov = 90;
    Vector3f backgroundColor = Vector3f(0.235294, 0.67451, 0.843137);
    int maxDepth = 5;
    float epsilon = 0.00001; // 用于避免浮点数精度问题

    Scene(int w, int h) : width(w), height(h)
    {}

    // 这里使用 std::unique_ptr 表示 Object
    // 对象的所有权只能有一个地方持有（即唯一所有权）。 当我们将
    // unique_ptr<Object> 作为参数传入时，意味着调用者放弃所有权，该对象归 Scene
    // 管理。 std::move(object) 把 unique_ptr 对象的所有权转移（移动）到 objects
    // 容器里。
    // 所有权转移后, 原持有对象会变成 nullptr, 不能再次使用
    void Add(std::unique_ptr<Object> object) {
        objects.push_back(std::move(object));
    }
    void Add(std::unique_ptr<Light> light) { lights.push_back(std::move(light)); }

    // [[nodiscard]] 用于提示调用者不要忽略返回值，否则编译器会发出警告。这样可以防止代码中忘记处理重要的返回对象。
    // 返回对场景内所有对象的常量引用（不能修改容器或对象指针本身，适用于只读访问，避免复制，提高效率）
    [[nodiscard]] const std::vector<std::unique_ptr<Object>>& get_objects() const { return objects; }
    [[nodiscard]] const std::vector<std::unique_ptr<Light> >&  get_lights() const { return lights; }

private:
    // creating the scene (adding objects and lights)
    std::vector<std::unique_ptr<Object> > objects;
    std::vector<std::unique_ptr<Light> > lights;
};
