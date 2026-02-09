#pragma once

#include "Object.hpp"

#include <cstring>

// 判断射线是否与三角形相交
// 参数说明：
// - const Vector3f& v0: 三角形顶点0，类型为 Vector3f。
// - const Vector3f& v1: 三角形顶点1，类型为 Vector3f。
// - const Vector3f& v2: 三角形顶点2，类型为 Vector3f。
// - const Vector3f& orig: 射线起点，类型为 Vector3f。
// - const Vector3f& dir: 射线方向，类型为 Vector3f。
// - float& tnear: 交点距离，类型为 float。
bool rayTriangleIntersect(const Vector3f &v0, const Vector3f &v1, const Vector3f &v2, const Vector3f &orig,
                          const Vector3f &dir, float &tnear, float &u, float &v)
{
    // TODO: Implement this function that tests whether the triangle
    // that's specified bt v0, v1 and v2 intersects with the ray (whose
    // origin is *orig* and direction is *dir*)
    // Also don't forget to update tnear, u and v.

    //代入课程公式,当t，b1，b2，(1-b1-b2)都大于0时，代表射线于三角形相交

    Vector3f O = orig, D = dir, P0 = v0, P1 = v1, P2 = v2;
    Vector3f E1 = P1 - P0, E2 = P2 - P0;
    Vector3f S = O - P0, S1 = crossProduct(D, E2), S2 = crossProduct(S, E1);

    tnear = dotProduct(S2, E2) / dotProduct(S1, E1);
    u = dotProduct(S1, S) / dotProduct(S1, E1);
    v = dotProduct(S2, D) / dotProduct(S1, E1);

    return tnear > 0 && u > 0 && v > 0 && (1 - u - v) > 0;
}

// 网格三角形类，继承自 Object 类
class MeshTriangle : public Object
{
public:
    // 构造函数，初始化顶点、索引、纹理坐标
    // 参数说明：
    // - const Vector3f *verts: 顶点数组，类型为 Vector3f 的指针。
    // - const uint32_t *vertsIndex: 顶点索引数组，类型为 uint32_t 的指针。
    // - const uint32_t &numTris: 三角形数量，类型为 uint32_t 的引用。
    // - const Vector2f *st: 纹理坐标数组，类型为 Vector2f 的指针。
    MeshTriangle(const Vector3f *verts, const uint32_t *vertsIndex, const uint32_t &numTris, const Vector2f *st)
    {
        uint32_t maxIndex = 0;
        for (uint32_t i = 0; i < numTris * 3; ++i)
            if (vertsIndex[i] > maxIndex) // 如果顶点索引大于最大索引，则更新最大索引
                maxIndex = vertsIndex[i]; // 更新最大索引
        maxIndex += 1; // 最大索引加1
        vertices = std::unique_ptr<Vector3f[]>(new Vector3f[maxIndex]); // 创建顶点数组
        memcpy(vertices.get(), verts, sizeof(Vector3f) * maxIndex); // 复制顶点数据
        vertexIndex = std::unique_ptr<uint32_t[]>(new uint32_t[numTris * 3]); // 创建顶点索引数组
        memcpy(vertexIndex.get(), vertsIndex, sizeof(uint32_t) * numTris * 3); // 复制顶点索引数据
        numTriangles = numTris; // 三角形数量
        stCoordinates = std::unique_ptr<Vector2f[]>(new Vector2f[maxIndex]); // 创建纹理坐标数组
        memcpy(stCoordinates.get(), st, sizeof(Vector2f) * maxIndex); // 复制纹理坐标数据
    }

    // 判断射线是否与三角形相交
    // 参数说明：
    // - const Vector3f &orig: 射线起点，类型为 Vector3f。
    // - const Vector3f &dir: 射线方向，类型为 Vector3f。
    // - float &tnear: 交点距离，类型为 float。
    // - uint32_t &index: 三角形索引，类型为 uint32_t。
    // - Vector2f &uv: 交点纹理坐标，类型为 Vector2f。
    bool intersect(const Vector3f &orig, const Vector3f &dir, float &tnear, uint32_t &index,
                   Vector2f &uv) const override
    {
        bool intersect = false;
        for (uint32_t k = 0; k < numTriangles; ++k) // 遍历每个三角形
        {
            const Vector3f &v0 = vertices[vertexIndex[k * 3]];
            const Vector3f &v1 = vertices[vertexIndex[k * 3 + 1]];
            const Vector3f &v2 = vertices[vertexIndex[k * 3 + 2]];
            float t, u, v;
            if (rayTriangleIntersect(v0, v1, v2, orig, dir, t, u, v) && t < tnear) // 如果射线与三角形相交，并且交点距离小于当前交点距离
            {
                tnear = t;
                uv.x = u; // 交点纹理坐标x
                uv.y = v; // 交点纹理坐标y
                index = k; // 三角形索引
                intersect |= true; // 交点标志位设置为true
            }
        }

        return intersect;
    }

    // 获取表面属性
    // 参数说明：
    // - const Vector3f &: 不使用，可忽略。
    // - const Vector3f &: 不使用，可忽略。
    // - const uint32_t &index: 三角形索引，类型为 uint32_t, 传入
    // - const Vector2f &uv: 交点纹理坐标，类型为 Vector2f, 返回
    // - Vector3f &N: 法线，类型为 Vector3f, 返回
    // - Vector2f &st: 交点纹理坐标，类型为 Vector2f, 返回
    void getSurfaceProperties(const Vector3f &, const Vector3f &, const uint32_t &index, const Vector2f &uv, Vector3f &N,
                              Vector2f &st) const override
    {
        const Vector3f &v0 = vertices[vertexIndex[index * 3]];      // 三角形顶点0
        const Vector3f &v1 = vertices[vertexIndex[index * 3 + 1]];  // 三角形顶点1
        const Vector3f &v2 = vertices[vertexIndex[index * 3 + 2]];  // 三角形顶点2
        Vector3f e0 = normalize(v1 - v0); // 向量e0 = 向量v1 - 向量v0
        Vector3f e1 = normalize(v2 - v1); // 向量e1 = 向量v2 - 向量v1
        N = normalize(crossProduct(e0, e1)); // 法线 = 向量e0和向量e1的叉积
        const Vector2f &st0 = stCoordinates[vertexIndex[index * 3]];     // 三角形顶点0的纹理坐标
        const Vector2f &st1 = stCoordinates[vertexIndex[index * 3 + 1]]; // 三角形顶点1的纹理坐标
        const Vector2f &st2 = stCoordinates[vertexIndex[index * 3 + 2]]; // 三角形顶点2的纹理坐标
        st = st0 * (1 - uv.x - uv.y) + st1 * uv.x + st2 * uv.y; // 交点纹理坐标 = 三角形顶点0的纹理坐标 * (1 - 交点纹理坐标x - 交点纹理坐标y) + 三角形顶点1的纹理坐标 * 交点纹理坐标x + 三角形顶点2的纹理坐标 * 交点纹理坐标y
    }

    // 计算漫反射颜色
    // 参数说明：
    // - const Vector2f &st: 交点纹理坐标，类型为 Vector2f。
    Vector3f evalDiffuseColor(const Vector2f &st) const override
    {
        float scale = 5; // 缩放因子
        // 棋盘格纹的的经典算法
        float pattern = (fmodf(st.x * scale, 1) > 0.5) ^ (fmodf(st.y * scale, 1) > 0.5); // 计算模式
        return lerp(Vector3f(0.815, 0.235, 0.031), Vector3f(0.937, 0.937, 0.231), pattern); // 插值计算漫反射颜色
    }

    std::unique_ptr<Vector3f[]> vertices; // 顶点数组
    uint32_t numTriangles; // 三角形数量
    std::unique_ptr<uint32_t[]> vertexIndex; // 顶点索引数组
    std::unique_ptr<Vector2f[]> stCoordinates; // 纹理坐标数组
};
