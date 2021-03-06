#include "sphere.h"
#include <iostream>
#include <cmath>
//--------------------------------------------------------------------------------
// icosahedron data
//--------------------------------------------------------------------------------
#define X .525731112119133606
#define Z .850650808352039932

static Point3f vdata[12] = {
    Point3f(-X, 0.0, Z), Point3f(X, 0.0, Z), Point3f(-X, 0.0, -Z), Point3f(X, 0.0, -Z),
    Point3f(0.0, Z, X), Point3f(0.0, Z, -X), Point3f(0.0, -Z, X), Point3f(0.0, -Z, -X),
    Point3f(Z, X, 0.0), Point3f(-Z, X, 0.0), Point3f(Z, -X, 0.0), Point3f(-Z, -X, 0.0)
};

static int tindices[20][3] = {
    {0,4,1}, {0,9,4}, {9,5,4}, {4,5,8}, {4,8,1},
    {8,10,1}, {8,3,10}, {5,3,8}, {5,2,3}, {2,7,3},
    {7,10,3}, {7,6,10}, {7,11,6}, {11,0,6}, {0,1,6},
    {6,1,10}, {9,0,11}, {9,11,2}, {9,2,5}, {7,2,11} };
//--------------------------------------------------------------------------------

void Sphere::subdivide(int i1, int i2, int i3, std::vector<Point3f> &sphere_points,
                       std::map< std::pair<int,int>, int> &dico, std::vector<int> &indices, const unsigned int depth) {
    if(depth == 0){
        indices.push_back(i2);
        indices.push_back(i1);
        indices.push_back(i3);
        return;
    }

    Point3f v1 = sphere_points[i1];
    Point3f v2 = sphere_points[i2];
    Point3f v3 = sphere_points[i3];

    Point3f v12, v23, v31;
    std::pair<int,int> i12_pair(std::min(i1,i2),std::max(i1,i2));
    if(dico.find(i12_pair) == dico.end()){
        dico[i12_pair] = sphere_points.size();
        sphere_points.push_back((v1 + v2).normalized());
    }
    int i12 = dico[i12_pair];
    v12 = sphere_points[i12];

    std::pair<int,int> i23_pair(std::min(i3,i2),std::max(i3,i2));
    if(dico.find(i23_pair) == dico.end()){
        dico[i23_pair] = sphere_points.size();
        sphere_points.push_back((v2 + v3).normalized());
    }
    int i23 = dico[i23_pair];
    v23 = sphere_points[i23];

    std::pair<int,int> i31_pair(std::min(i1,i3),std::max(i1,i3));
    if(dico.find(i31_pair) == dico.end()){
        dico[i31_pair] = sphere_points.size();
        sphere_points.push_back((v3 + v1).normalized());
    }
    int i31 = dico[i31_pair];
    v31 = sphere_points[i31];

    subdivide(i1, i12, i31, sphere_points, dico, indices, depth - 1);
    subdivide(i2, i23, i12, sphere_points, dico, indices, depth - 1);
    subdivide(i3, i31, i23, sphere_points, dico, indices, depth - 1);
    subdivide(i12, i23, i31, sphere_points, dico, indices, depth - 1);
}

Sphere::Sphere(float radius)
    : m_radius(radius)
{
    init();
}

Sphere::Sphere(const PropertyList &propList)
{
    m_radius = propList.getFloat("radius",1.f);
    m_transformation = propList.getTransform("toWorld", Transform());
    init();
}

void Sphere::init()
{
    m_pMesh = new Mesh;

    std::vector<Point3f> sphere_points;
    for(int i = 0; i < 12; i++){
        sphere_points.push_back(vdata[i]);
    }
    std::vector<int> indices;
    std::map< std::pair<int,int>, int> dico;
    for(int i = 0; i < 20; i++){
        subdivide(tindices[i][0], tindices[i][1], tindices[i][2], sphere_points, dico, indices, 3);
    }

    float vertices[3*sphere_points.size()];
    for(uint i=0; i<sphere_points.size(); i++){
        vertices[i*3  ] = sphere_points[i][0] * m_radius;
        vertices[i*3+1] = sphere_points[i][1] * m_radius;
        vertices[i*3+2] = sphere_points[i][2] * m_radius;
    }

    m_pMesh->loadRawData((float*)vertices, sphere_points.size(), (int*)&indices[0], indices.size()/3);
}

Sphere::~Sphere()
{
    delete m_pMesh;
}

void Sphere::drawGeometry() const
{
    m_pMesh->drawGeometry();
}

const Eigen::AlignedBox3f& Sphere::AABB() const
{
    return m_pMesh->AABB();
}

bool Sphere::intersect(const Ray& ray, Hit& hit) const
{
    /// TODO: compute ray-sphere intersection
    float Delta;
    Vector3f O = ray.origin;
    Vector3f D = ray.direction;
    float a = D.squaredNorm();
    float b = 2*O.dot(D);
    Delta = pow(b,2)-4*a*(O.squaredNorm()-pow(m_radius,2));
    if (Delta<0)
            return false;
    else{
        float A = (-b-sqrt(Delta))/2.0/a;
        float B = (-b+sqrt(Delta))/2.0/a;
        if (A<=0){
            if(B<=0)
                return false;
            else{
                hit.setT(B);
                return true;}}
        else{
            if (B<=0){
                hit.setT(A);
               return true;}
            else{
                if (A<B){
                    hit.setT(A);
                    return true;}
                else{
                    hit.setT(B);
                    return true;}
            }
       }
    }

    //throw SireException("Sphere::intersect not implemented yet.");

}

REGISTER_CLASS(Sphere, "sphere")
