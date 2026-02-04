#include <fstream>
#include "Vector.hpp"
#include "Renderer.hpp"
#include "Scene.hpp"
#include <optional>

// 将角度转换为弧度
inline float deg2rad(const float &deg)
{
    return deg * M_PI / 180.0;
}

// 计算反射方向
Vector3f reflect(const Vector3f &I, const Vector3f &N)
{
    return I - 2 * dotProduct(I, N) * N;
}

// [comment]
// Compute refraction direction using Snell's law
//
// We need to handle with care the two possible situations:
//
//    - When the ray is inside the object
//    - When the ray is outside.
//
// If the ray is outside, you need to make cosi positive cosi = -N.I
// If the ray is inside, you need to invert the refractive indices and negate
// the normal N
// [/comment]
// 使用斯涅尔定律计算折射方向
// 参数说明：
// - const Vector3f &I: 入射方向
// - const Vector3f &N: 法线
// - const float &ior: 折射率
// 返回值：
// - Vector3f: 折射方向
// 计算过程：
// 1. 计算入射方向与法线的点积
// 2. 计算折射率
// 3. 计算折射方向
// 4. 返回折射方向
Vector3f refract(const Vector3f &I, const Vector3f &N, const float &ior)
{
    float cosi = clamp(-1, 1, dotProduct(I, N)); // 计算入射方向与法线的点积，并限制在[-1, 1]之间

    // etai 表示入射介质（incident medium）的折射率
    // etat 表示透射介质（transmission medium）的折射率
    // 光从空气进入玻璃时：etai = 1（空气），etat = ior（物体的折射率）
    // 光从物体内部射向空气时：etai = ior，etat = 1
    float etai = 1.0f;  // 入射介质的折射率（通常为空气，设为1）
    float etat = ior;   // 透射介质的折射率（物体自身的折射率）

    Vector3f n = N; // 初始化法线

    if (cosi < 0)
    {
        // 光线在物体外部（从空气射向物体）
        cosi = -cosi; // 取正值
    }
    else
    {
        // 光线在物体内部（从物体射向空气）
        std::swap(etai, etat); // 交换折射率，使得入射介质和透射介质顺序正确
        n = -N; // 法线取反
    }
    float eta = etai / etat; // 计算相对折射率
    float k = 1 - eta * eta * (1 - cosi * cosi); // 计算折射方向是否存在（k<0表示全反射）
    return k < 0 ? 0 : eta * I + (eta * cosi - sqrtf(k)) * n; // 返回折射方向
}

// [comment]
// Compute Fresnel equation
//
// \param I is the incident view direction
//
// \param N is the normal at the intersection point
//
// \param ior is the material refractive index
// [/comment]

// 计算菲涅尔方程
// 参数说明：
// - const Vector3f &I: 入射方向
// - const Vector3f &N: 法线
// - const float &ior: 折射率
// 返回值：
// - float: 菲涅尔方程的值
// 计算过程：
// 1. 计算入射方向与法线的点积
// 2. 计算折射率
// 3. 计算菲涅尔方程的值
// 4. 返回菲涅尔方程的值
float fresnel(const Vector3f &I, const Vector3f &N, const float &ior)
{
    float cosi = clamp(-1, 1, dotProduct(I, N)); // 计算入射方向与法线的点积，并限制在[-1, 1]之间
    float etai = 1, etat = ior; // 初始化折射率
    if (cosi > 0)
    {
        std::swap(etai, etat); // 交换折射率，使得入射介质和透射介质顺序正确
    }
    // Compute sini using Snell's law
    float sint = etai / etat * sqrtf(std::max(0.f, 1 - cosi * cosi)); // 计算折射方向是否存在（sint>=1表示全反射）
    // Total internal reflection
    if (sint >= 1) // 如果折射方向不存在，则返回1
    {
        return 1; // 返回1，表示全反射
    }
    else
    {
        float cost = sqrtf(std::max(0.f, 1 - sint * sint)); // 计算折射方向是否存在（cost<0表示全反射）
        cosi = fabsf(cosi);
        float Rs = ((etat * cosi) - (etai * cost)) / ((etat * cosi) + (etai * cost)); // 计算反射光的强度
        float Rp = ((etai * cosi) - (etat * cost)) / ((etai * cosi) + (etat * cost)); // 计算折射光的强度
        return (Rs * Rs + Rp * Rp) / 2; // 返回反射和折射光的强度
    }
    // As a consequence of the conservation of energy, transmittance is given by:
    // kt = 1 - kr;
}

// [comment]
// Returns true if the ray intersects an object, false otherwise.
//
// \param orig is the ray origin
// \param dir is the ray direction
// \param objects is the list of objects the scene contains
// \param[out] tNear contains the distance to the cloesest intersected object.
// \param[out] index stores the index of the intersect triangle if the
// interesected object is a mesh.
// \param[out] uv stores the u and v barycentric coordinates of the intersected
// point
// \param[out] *hitObject stores the pointer to the intersected object (used to
// retrieve material information, etc.)
// \param isShadowRay is it a shadow ray. We can return from the function sooner
// as soon as we have found a hit.
// [/comment]

// 追踪光线与物体相交
// 参数说明：
// - const Vector3f &orig: 光线起点
// - const Vector3f &dir: 光线方向
// - const std::vector<std::unique_ptr<Object>> &objects: 物体列表
// 返回值：
// - std::optional<hit_payload>: 光线与物体相交的信息
// 计算过程：
// 1. 初始化距离为无穷大
// 2. 遍历物体列表
// 3. 计算光线与物体相交的距离
// 4. 更新光线与物体相交的距离
// 5. 返回光线与物体相交的信息
std::optional<hit_payload> trace(
    const Vector3f &orig, const Vector3f &dir,
    const std::vector<std::unique_ptr<Object>> &objects)
{
    float tNear = kInfinity; // 初始化距离为无穷大
    std::optional<hit_payload> payload; // 初始化光线与物体相交的信息
    for (const auto &object : objects)
    {
        float tNearK = kInfinity; // 初始化距离为无穷大
        uint32_t indexK; // 初始化物体索引
        Vector2f uvK; // 初始化纹理坐标
        if (object->intersect(orig, dir, tNearK, indexK, uvK) && tNearK < tNear) // 如果光线与物体相交，并且交点距离小于当前交点距离
        {
            // emplace 的作用是「就地构造」optional/hit_payload 对象，将其变为有效状态，而无需先创建临时对象再赋值，提高效率。
            // 这里等价于 payload = hit_payload{}，但 emplace 用于可选类型或容器时效率更高，也可以传递构造参数。
            payload.emplace(); // 就地初始化 payload
            payload->hit_obj = object.get(); // 设置光线与物体相交的物体
            payload->tNear = tNearK; // 设置光线与物体相交的距离
            payload->index = indexK; // 设置光线与物体相交的物体索引
            payload->uv = uvK; // 设置光线与物体相交的纹理坐标
            tNear = tNearK; // 设置光线与物体相交的距离
        }
    }

    return payload; // 返回光线与物体相交的信息
}

// [comment]
// Implementation of the Whitted-style light transport algorithm (E [S*] (D|G) L)
//
// This function is the function that compute the color at the intersection point
// of a ray defined by a position and a direction. Note that thus function is recursive (it calls itself).
//
// If the material of the intersected object is either reflective or reflective and refractive,
// then we compute the reflection/refraction direction and cast two new rays into the scene
// by calling the castRay() function recursively. When the surface is transparent, we mix
// the reflection and refraction color using the result of the fresnel equations (it computes
// the amount of reflection and refraction depending on the surface normal, incident view direction
// and surface refractive index).
//
// If the surface is diffuse/glossy we use the Phong illumation model to compute the color
// at the intersection point.
// [/comment]

// Whitted-style光线追踪
// 参数说明：
// - const Vector3f &orig: 光线起点
// - const Vector3f &dir: 光线方向
// - const Scene &scene: 场景
// - int depth: 光线深度
// 返回值：
// - Vector3f: 光线颜色
// 计算过程：
// 1. 如果光线深度大于最大深度，则返回背景颜色
// 2. 初始化光线颜色为背景颜色
// 3. 计算光线与物体相交的信息
// 4. 计算光线与物体相交的点
// 5. 计算光线与物体相交的法线
// 6. 计算光线与物体相交的纹理坐标
// 7. 根据物体材质类型计算光线颜色
// 8. 返回光线颜色
Vector3f castRay(
    const Vector3f &orig, const Vector3f &dir, const Scene &scene,
    int depth)
{
    if (depth > scene.maxDepth) // 最大递归深度, 如果光线深度大于最大深度，则返回背景颜色
    {
        return Vector3f(0.0, 0.0, 0.0); // 返回背景颜色
    }

    Vector3f hitColor = scene.backgroundColor; // 初始化光线颜色为背景颜色
    // 这是 C++17 的 if 语句加强语法，称为 "if 带初始化语句（if with initializer statement）"。
    // auto payload = trace(orig, dir, scene.get_objects()) 在 if 作用域内部定义并初始化变量 payload，
    // 并在 payload 有值（即 payload 转换为 true，通常是 std::optional、智能指针等可判断真假类型）时进入 if 块。
    if (auto payload = trace(orig, dir, scene.get_objects()); payload)
    {
        Vector3f hitPoint = orig + dir * payload->tNear; // 计算光线与物体相交的点
        Vector3f N;  // 法线
        Vector2f st; // 交点纹理坐标
        payload->hit_obj->getSurfaceProperties(hitPoint, dir, payload->index, payload->uv, N, st); // 计算光线与物体相交的法线
        switch (payload->hit_obj->materialType) // 根据物体材质类型计算光线颜色
        {
        case REFLECTION_AND_REFRACTION: // 反射和折射
        {
            Vector3f reflectionDirection = normalize(reflect(dir, N)); // 计算反射方向
            Vector3f refractionDirection = normalize(
                refract(dir, N, payload->hit_obj->ior)); // 计算折射方向
            // 计算反射的起点
            Vector3f reflectionRayOrig =
                (dotProduct(reflectionDirection, N) < 0)
                    ? hitPoint - N * scene.epsilon
                    : hitPoint + N * scene.epsilon;
            // 计算折射的起点
            Vector3f refractionRayOrig =
                (dotProduct(refractionDirection, N) < 0)
                    ? hitPoint - N * scene.epsilon
                    : hitPoint + N * scene.epsilon;
            // 计算反射的颜色
            Vector3f reflectionColor = castRay(reflectionRayOrig, reflectionDirection, scene, depth + 1); // 递归计算
            // 计算折射的颜色
            Vector3f refractionColor = castRay(refractionRayOrig, refractionDirection, scene, depth + 1); // 递归计算
            // 计算菲涅尔方程
            float kr = fresnel(dir, N, payload->hit_obj->ior);
            // 计算反射和折射的颜色
            hitColor = reflectionColor * kr + refractionColor * (1 - kr);
            break;
        }
        case REFLECTION: {
            // 计算菲涅尔方程
            float kr = fresnel(dir, N, payload->hit_obj->ior);
            Vector3f reflectionDirection = reflect(dir, N); // 计算反射方向
            // 计算反射的起点
            Vector3f reflectionRayOrig = (dotProduct(reflectionDirection, N) < 0) ? hitPoint + N * scene.epsilon : hitPoint - N * scene.epsilon;
            // 计算反射的颜色
            hitColor = castRay(reflectionRayOrig, reflectionDirection, scene, depth + 1) * kr; // 递归计算反射的颜色
            break;
        }
        default:
        {
            // [comment]
            // We use the Phong illumation model int the default case. The phong model
            // is composed of a diffuse and a specular reflection component.
            // [/comment]
            // 累积的漫反射光照强度 和 累积的镜面反射颜色
            Vector3f lightAmt = 0, specularColor = 0;
            // 计算阴影的起点

            // 从交点沿法线方向偏移 epsilon
            // 若视线方向与法线夹角 > 90°（点在背面），起点在法线正方向；否则在负方向
            Vector3f shadowPointOrig = (dotProduct(dir, N) < 0) ? hitPoint + N * scene.epsilon : hitPoint - N * scene.epsilon;
            // [comment]
            // Loop over all lights in the scene and sum their contribution up
            // We also apply the lambert cosine law
            // [/comment]

            // 遍历所有光源
            for (auto &light : scene.get_lights())
            {
                Vector3f lightDir = light->position - hitPoint; // 计算光源方向
                // square of the distance between hitPoint and the light
                float lightDistance2 = dotProduct(lightDir, lightDir); // 计算光源距离的平方
                lightDir = normalize(lightDir); // 计算光源方向

                // 计算漫反射（Lambert 定律）L·N（光源方向与法线的点积）
                // 光照强度与入射角余弦成正比
                // std::max(0.f, ...)：背面不接收光照
                float LdotN = std::max(0.f, dotProduct(lightDir, N));
                // is the point in shadow, and is the nearest occluding object
                // closer to the object than the light itself?
                // 阴影检测
                // 从偏移起点向光源方向发射阴影光线
                // 若命中物体且距离小于到光源的距离，则点在阴影中
                auto shadow_res = trace(shadowPointOrig, lightDir, scene.get_objects()); // 计算阴影的起点
                bool inShadow = shadow_res && (shadow_res->tNear * shadow_res->tNear < lightDistance2); // 计算阴影的起点是否在阴影中
                // 累积漫反射强度
                lightAmt += inShadow ? 0 : light->intensity * LdotN;
                // 计算光线反射方向（注意取反）
                Vector3f reflectionDirection = reflect(-lightDir, N);
                // 计算镜面反射（Phong 高光）
                // -dotProduct(reflectionDirection,
                // dir)：反射方向与视线方向的点积（视线方向取反）
                // powf(..., specularExponent)：高光衰减，指数越大高光越集中
                // 视线越接近反射方向，高光越强
                specularColor += powf(std::max(0.f, -dotProduct(reflectionDirection, dir)),
                                      payload->hit_obj->specularExponent) *
                                 light->intensity; // 计算镜面反射的强度
            }

            hitColor = lightAmt * payload->hit_obj->evalDiffuseColor(st) * payload->hit_obj->Kd + specularColor * payload->hit_obj->Ks; // 计算光线颜色
            break;
        }
        }
    }

    return hitColor; // 返回光线颜色
}

// [comment]
// The main render function. This where we iterate over all pixels in the image, generate
// primary rays and cast these rays into the scene. The content of the framebuffer is
// saved to a file.
// [/comment]
void Renderer::Render(const Scene &scene)
{
    std::vector<Vector3f> framebuffer(scene.width * scene.height);

    float scale = std::tan(deg2rad(scene.fov * 0.5f));
    float imageAspectRatio = scene.width / (float)scene.height;

    // Use this variable as the eye position to start your rays.
    Vector3f eye_pos(0);
    int m = 0;
    for (int j = 0; j < scene.height; ++j)
    {
        for (int i = 0; i < scene.width; ++i)
        {
            // generate primary ray direction
            // TODO: Find the x and y positions of the current pixel to get the direction
            // vector that passes through it.
            // Also, don't forget to multiply both of them with the variable *scale*, and
            // x (horizontal) variable with the *imageAspectRatio*

            //https://blog.csdn.net/dong89801033/article/details/114834898?ops_request_misc=%257B%2522request%255Fid%2522%253A%2522162216944616780357298394%2522%252C%2522scm%2522%253A%252220140713.130102334.pc%255Fall.%2522%257D&request_id=162216944616780357298394&biz_id=0&utm_medium=distribute.pc_search_result.none-task-blog-2~all~first_rank_v2~rank_v29-2-114834898.pc_search_result_cache&utm_term=games101%E4%BD%9C%E4%B8%9A5&spm=1018.2226.3001.4187
            //  +0.5是取像素中间的光线
            //  ((float)i + 0.5)/scene.width 和 ((float)j + 0.5) / scene.height 是把屏幕转换到为以左上角为原点，x和y轴最大值为1的坐标系中
            //  2*(x - 1), 2*(y - 1)*-1 是把坐标系原点平移到中心，也就是转换到以屏幕中心为原点，[-1，1]的坐标系，y轴需要从向下转换成向上，所以乘以-1
            float x = (2 * (((float)i + 0.5) / scene.width) - 1) * scale * imageAspectRatio;
            float y = (2 * (((float)j + 0.5) / scene.height) - 1) * -1 * scale;

            Vector3f dir = normalize(Vector3f(x, y, -1)); // Don't forget to normalize this direction!
            framebuffer[m++] = castRay(eye_pos, dir, scene, 0);
        }
        UpdateProgress(j / (float)scene.height);
    }

    // save framebuffer to file
    FILE *fp = fopen("binary.ppm", "wb");
    (void)fprintf(fp, "P6\n%d %d\n255\n", scene.width, scene.height);
    for (auto i = 0; i < scene.height * scene.width; ++i)
    {
        static unsigned char color[3];
        color[0] = (char)(255 * clamp(0, 1, framebuffer[i].x));
        color[1] = (char)(255 * clamp(0, 1, framebuffer[i].y));
        color[2] = (char)(255 * clamp(0, 1, framebuffer[i].z));
        fwrite(color, 1, 3, fp);
    }
    fclose(fp);
}
