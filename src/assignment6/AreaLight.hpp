//
// Created by Göksu Güvendiren on 2019-05-14.
//

#pragma once

#include "Vector.hpp"
#include "Light.hpp"
#include "global.hpp"

class AreaLight : public Light
{
public:

    /**
     * @brief Constructs an AreaLight with the specified position and intensity.
     *
     * Initializes an area light source with a rectangular emission area defined by
     * two orthogonal basis vectors (u and v). The light emits uniformly across this
     * rectangular surface.
     *
     * @param p The position (center) of the area light in world space.
     * @param i The intensity (radiance) of the light source.
     *
     * @note Member variables initialized:
     *   - normal: Surface normal vector pointing downward (0, -1, 0), indicating
     *     the light emits primarily in the negative Y direction.
     *   - u: Basis vector along the X-axis (1, 0, 0), defines one edge direction
     *     of the rectangular light surface. Used to parameterize the light's width.
     *   - v: Basis vector along the Z-axis (0, 0, 1), defines the other edge direction
     *     of the rectangular light surface. Used to parameterize the light's depth.
     *   - length: The size parameter (100 units) controlling the dimensions of the
     *     rectangular light surface (typically length × length area).
     *
     * @details u和v的作用:
     *   u和v是构成矩形面光源的两个正交基向量。它们共同定义了一个矩形光源表面的方向和范围。
     *   在光线追踪中，可以使用u和v参数化出光源表面上的任意点：
     *   点 = p + s*u*length + t*v*length，其中s, t ∈ [0, 1]
     *   这使得可以实现对面光源的重要性采样。
     */
    AreaLight(const Vector3f &p, const Vector3f &i) : Light(p, i)
    {
        normal = Vector3f(0, -1, 0);
        u = Vector3f(1, 0, 0);
        v = Vector3f(0, 0, 1);
        length = 100;
    }

    /// \brief Samples a random point on the area light surface
    /// \details Generates a uniformly distributed random point within the rectangular area
    ///          defined by the light's position and two edge vectors (u and v).
    ///          Uses two independent random values [0,1) to compute barycentric-like coordinates.
    /// \return A Vector3f representing the sampled point in world space
    /// \note This function is commonly used in Monte Carlo path tracing for direct lighting estimation
    Vector3f SamplePoint() const
    {
        auto random_u = get_random_float();
        auto random_v = get_random_float();
        return position + random_u * u + random_v * v;
    }

    float length;           // 面光源边长，即 u, v 方向的长度 (假设是正方形面光源)
    Vector3f normal;        // 面光源的法向量，指示光源朝向
    Vector3f u;             // 表示面光源上第一条边的向量（与 normal 垂直）
    Vector3f v;             // 表示面光源上第二条边的向量（与 normal 和 u 都垂直），与 u 一起定义面的位置与朝向
};
