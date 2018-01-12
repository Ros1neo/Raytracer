#ifndef SIRE_RAY
#define SIRE_RAY

#include "common.h"
#include "vector.h"

class Shape;

class Ray
{
public:
    Ray(const Point3f& o, const Vector3f& d)
        : origin(o), direction(d), recursionLevel(0), shadowRay(false)
    {}
    Ray() : recursionLevel(0), shadowRay(false) {}

    Point3f origin;
    Vector3f direction;

    Point3f at(float t) const { return origin + t*direction; }

    int recursionLevel;   ///< recursion level (used as a stoping critera)
    bool shadowRay;       ///< tag for shadow rays
};

class Hit
{
public:
    Hit() : m_texcoord(0,0), mp_shape(0), m_t(std::numeric_limits<float>::max()) {}

    bool foundIntersection() const { return m_t < std::numeric_limits<float>::max(); }

    void setT(float t) { m_t = t; }
    float t() const { return m_t; }

    void setIntersection(const Point3f& i) { m_intersection = i; }
    const Point3f& intersection() { return m_intersection; }

    void setNormal(const Vector3f& normal) { m_normal = normal; }
    const Vector3f& normal() const { return m_normal; }

    void setShape(const Shape* shape) { mp_shape = shape; }
    const Shape* shape() const { return mp_shape; }

    void setTexcoord(const Vector2f& uv) { m_texcoord = uv; }
    const Vector2f& texcoord() const { return m_texcoord; }

private:
    Point3f m_intersection;
    Vector3f m_normal;
    Vector2f m_texcoord;
    const Shape* mp_shape;
    float m_t;
};

/** Compute the intersection between a ray and an aligned box
  * \returns true if an intersection is found
  * The ranges are returned in tMin,tMax
  */
static inline bool intersect(const Ray& ray, const Eigen::AlignedBox3f& box, float& tMin, float& tMax, Normal3f& normal)
{
    Eigen::Array3f t1, t2;
    t1 = (box.min()-ray.origin).cwiseQuotient(ray.direction);
    t2 = (box.max()-ray.origin).cwiseQuotient(ray.direction);
    Eigen::Array3f::Index maxIdx, minIdx;
    tMin = t1.min(t2).maxCoeff(&maxIdx);
    tMax = t1.max(t2).minCoeff(&minIdx);
    normal = Normal3f::Zero();
    normal[maxIdx] = -1;
    return tMax>0 && tMin<=tMax;
}

#endif
