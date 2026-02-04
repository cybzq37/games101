#include "Scene.hpp"
#include "Sphere.hpp"
#include "Triangle.hpp"
#include "Light.hpp"
#include "Renderer.hpp"

// In the main function of the program, we create the scene (create objects and lights)
// as well as set the options for the render (image width and height, maximum recursion
// depth, field-of-view, etc.). We then call the render function().
int main()
{
    Scene scene(1280, 960); // 创建场景，宽1280，高960

    auto sph1 = std::make_unique<Sphere>(Vector3f(-1, 0, -12), 2); // 创建球体，位置(-1, 0, -12)，半径2
    sph1->materialType = DIFFUSE_AND_GLOSSY; // 设置球体材质为漫反射和光泽
    sph1->diffuseColor = Vector3f(0.6, 0.7, 0.8); // 设置球体漫反射颜色为(0.6, 0.7, 0.8)

    auto sph2 = std::make_unique<Sphere>(Vector3f(0.5, -0.5, -8), 1.5); // 创建球体，位置(0.5, -0.5, -8)，半径1.5
    sph2->ior = 1.5; // 设置球体折射率为1.5
    sph2->materialType = REFLECTION_AND_REFRACTION; // 设置球体材质为反射和折射

    scene.Add(std::move(sph1)); // 添加球体到场景
    scene.Add(std::move(sph2)); // 添加球体到场景

    Vector3f verts[4] = {{-5,-3,-6}, {5,-3,-6}, {5,-3,-16}, {-5,-3,-16}}; // 创建四个顶点，位置分别为(-5, -3, -6)，(5, -3, -6)，(5, -3, -16)，(-5, -3, -16)
    uint32_t vertIndex[6] = {0, 1, 3, 1, 2, 3}; // 创建六个顶点索引，分别为0, 1, 3, 1, 2, 3
    Vector2f st[4] = {{0, 0}, {1, 0}, {1, 1}, {0, 1}}; // 创建四个纹理坐标，分别为(0, 0)，(1, 0)，(1, 1)，(0, 1)
    auto mesh = std::make_unique<MeshTriangle>(verts, vertIndex, 2, st); // 创建网格，顶点数组，顶点索引数组，纹理坐标数组
    mesh->materialType = DIFFUSE_AND_GLOSSY; // 设置网格材质为漫反射和光泽

    scene.Add(std::move(mesh)); // 添加网格到场景
    scene.Add(std::make_unique<Light>(Vector3f(-20, 70, 20), 0.5)); // 添加光源到场景，位置(-20, 70, 20)，强度0.5
    scene.Add(std::make_unique<Light>(Vector3f(30, 50, -12), 0.5)); // 添加光源到场景，位置(30, 50, -12)，强度0.5

    Renderer r; // 创建渲染器
    r.Render(scene);

    return 0; // 返回0，表示程序正常结束
}
