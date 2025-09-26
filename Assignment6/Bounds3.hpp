//
// Created by LEI XU on 5/16/19.
//

#ifndef RAYTRACING_BOUNDS3_H
#define RAYTRACING_BOUNDS3_H
#include "Ray.hpp"
#include "Vector.hpp"
#include <limits>
#include <array>

class Bounds3
{
  public:
    Vector3f pMin, pMax; // two points to specify the bounding box
    Bounds3()
    {
        double minNum = std::numeric_limits<double>::lowest();
        double maxNum = std::numeric_limits<double>::max();
        pMax = Vector3f(minNum, minNum, minNum);
        pMin = Vector3f(maxNum, maxNum, maxNum);
    }
    Bounds3(const Vector3f p) : pMin(p), pMax(p) {}
    Bounds3(const Vector3f p1, const Vector3f p2)
    {
        pMin = Vector3f(fmin(p1.x, p2.x), fmin(p1.y, p2.y), fmin(p1.z, p2.z));
        pMax = Vector3f(fmax(p1.x, p2.x), fmax(p1.y, p2.y), fmax(p1.z, p2.z));
    }

    Vector3f Diagonal() const { return pMax - pMin; }
    int maxExtent() const
    {
        Vector3f d = Diagonal();
        if (d.x > d.y && d.x > d.z)
            return 0;
        else if (d.y > d.z)
            return 1;
        else
            return 2;
    }

    double SurfaceArea() const
    {
        Vector3f d = Diagonal();
        return 2 * (d.x * d.y + d.x * d.z + d.y * d.z);
    }

    Vector3f Centroid() { return 0.5 * pMin + 0.5 * pMax; }
    Bounds3 Intersect(const Bounds3& b)
    {
        return Bounds3(Vector3f(fmax(pMin.x, b.pMin.x), fmax(pMin.y, b.pMin.y),
                                fmax(pMin.z, b.pMin.z)),
                       Vector3f(fmin(pMax.x, b.pMax.x), fmin(pMax.y, b.pMax.y),
                                fmin(pMax.z, b.pMax.z)));
    }

    Vector3f Offset(const Vector3f& p) const
    {
        Vector3f o = p - pMin;
        if (pMax.x > pMin.x)
            o.x /= pMax.x - pMin.x;
        if (pMax.y > pMin.y)
            o.y /= pMax.y - pMin.y;
        if (pMax.z > pMin.z)
            o.z /= pMax.z - pMin.z;
        return o;
    }

    bool Overlaps(const Bounds3& b1, const Bounds3& b2)
    {
        bool x = (b1.pMax.x >= b2.pMin.x) && (b1.pMin.x <= b2.pMax.x);
        bool y = (b1.pMax.y >= b2.pMin.y) && (b1.pMin.y <= b2.pMax.y);
        bool z = (b1.pMax.z >= b2.pMin.z) && (b1.pMin.z <= b2.pMax.z);
        return (x && y && z);
    }

    bool Inside(const Vector3f& p, const Bounds3& b)
    {
        return (p.x >= b.pMin.x && p.x <= b.pMax.x && p.y >= b.pMin.y &&
                p.y <= b.pMax.y && p.z >= b.pMin.z && p.z <= b.pMax.z);
    }
    inline const Vector3f& operator[](int i) const
    {
        return (i == 0) ? pMin : pMax;
    }

    inline bool IntersectP(const Ray& ray, const Vector3f& invDir,
                           const std::array<int, 3>& dirisNeg) const;
};



inline bool Bounds3::IntersectP(const Ray& ray, const Vector3f& invDir,
                                const std::array<int, 3>& dirIsNeg) const
{
    // invDir: ray direction(x,y,z), invDir=(1.0/x,1.0/y,1.0/z), use this because Multiply is faster that Division
    // dirIsNeg: ray direction(x,y,z), dirIsNeg=[int(x>0),int(y>0),int(z>0)], use this to simplify your logic
    // TODO test if ray bound intersects
    // float tMin = (pMin.x - ray.origin.x) * invDir.x;
    // float tMax = (pMax.x - ray.origin.x) * invDir.x;    
    // if(!dirIsNeg[0])
    // {
    //     std::swap(tMin, tMax);        
    // }

    // float tyMin = (pMin.y - ray.origin.y) * invDir.y;
    // float tyMax = (pMax.y - ray.origin.y) * invDir.y;
    // if(!dirIsNeg[1])
    // {
    //     std::swap(tyMin, tyMax);        
    // }

    // if(tMin > tyMax || tyMin > tMax)
    // {
    //     return false;
    // }
    // tMin = std::max(tMin, tyMin);
    // tMax = std::min(tMax, tyMax);

    // float tzMin = (pMin.z - ray.origin.z) * invDir.z;
    // float tzMax = (pMax.z - ray.origin.z) * invDir.z;
    // if(!dirIsNeg[2])
    // {
    //     std::swap(tzMin, tzMax);        
    // }

    // if(tMin > tzMax || tzMin > tMax)
    // {
    //     return false;
    // }
    // tMin = std::max(tMin, tzMin);
    // tMax = std::min(tMax, tzMax);

    // return tMin < ray.t_max && tMax > ray.t_min;
    // 计算光线与各轴最小、最大平面的交点参数 t
    Vector3f tMin = (pMin - ray.origin) * invDir;
    Vector3f tMax = (pMax - ray.origin) * invDir;

    // 根据方向符号调整 tMin 和 tMax，确保 tMin < tMax
    if (!dirIsNeg[0]) std::swap(tMin.x, tMax.x); // 如果x方向为负，交换
    if (!dirIsNeg[1]) std::swap(tMin.y, tMax.y); // 如果y方向为负，交换
    if (!dirIsNeg[2]) std::swap(tMin.z, tMax.z); // 如果z方向为负，交换

    // 计算整体进入时间（最晚的进入时间）和整体离开时间（最早的离开时间）
    float tEnter = std::max(tMin.x, std::max(tMin.y, tMin.z));
    float tExit = std::min(tMax.x, std::min(tMax.y, tMax.z));

    // 判断是否相交：进入时间小于离开时间，且离开时间非负（光线向前传播）
    return (tEnter <= tExit) && (tExit >= 0);

}

inline Bounds3 Union(const Bounds3& b1, const Bounds3& b2)
{
    Bounds3 ret;
    ret.pMin = Vector3f::Min(b1.pMin, b2.pMin);
    ret.pMax = Vector3f::Max(b1.pMax, b2.pMax);
    return ret;
}

inline Bounds3 Union(const Bounds3& b, const Vector3f& p)
{
    Bounds3 ret;
    ret.pMin = Vector3f::Min(b.pMin, p);
    ret.pMax = Vector3f::Max(b.pMax, p);
    return ret;
}

#endif // RAYTRACING_BOUNDS3_H
