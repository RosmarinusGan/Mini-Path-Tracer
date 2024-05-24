#pragma once

#include "BVH.hpp"
#include "Intersection.hpp"
#include "Material.hpp"
#include "OBJ_Loader.hpp"
#include "Object.hpp"
#include "Triangle.hpp"
#include <cassert>
#include <array>


class MeshTriangle : public Object
{
public:
    Bounds3 bounding_box;
    std::unique_ptr<Vector3f[]> vertices;
    uint32_t numTriangles;
    std::unique_ptr<uint32_t[]> vertexIndex;
    std::unique_ptr<Vector2f[]> stCoordinates;

    std::vector<Triangle> triangles;

    BVHAccel* bvh;
    float area;

    Material* m;

    MeshTriangle(const std::string &filename, Material *mt,
                 Vector3f Trans = Vector3f(0.0, 0.0, 0.0), Vector3f Scale = Vector3f(1.0, 1.0, 1.0),
                 Vector3f xr = Vector3f(1.0, 0, 0), Vector3f yr = Vector3f(0, 1.0, 0), Vector3f zr = Vector3f(0, 0, 1))
    {
        // 从文件中读取model，分别存储三角形，其顶点，以及顶点索引，纹理坐标
        objl::Loader loader;
        loader.LoadFile(filename);
        area = 0;
        m = mt;
        assert(loader.LoadedMeshes.size() == 1);
        auto mesh = loader.LoadedMeshes[0];

        Vector3f min_vert = Vector3f{std::numeric_limits<float>::infinity(),
                                     std::numeric_limits<float>::infinity(),
                                     std::numeric_limits<float>::infinity()};
        Vector3f max_vert = Vector3f{-std::numeric_limits<float>::infinity(),
                                     -std::numeric_limits<float>::infinity(),
                                     -std::numeric_limits<float>::infinity()};
        for (int i = 0; i < mesh.Vertices.size(); i += 3) {
            std::array<Vector3f, 3> face_vertices;
            std::array<Vector2f, 3> face_textures;
            std::array<Vector3f, 3> vertices_normal;

            for (int j = 0; j < 3; j++) {
                auto vert = Vector3f(mesh.Vertices[i + j].Position.X,
                                     mesh.Vertices[i + j].Position.Y,
                                     mesh.Vertices[i + j].Position.Z);

                vert.x = dotProduct(vert, xr);
                vert.y = dotProduct(vert, yr);
                vert.z = dotProduct(vert, zr);//旋转
                vert = Scale * vert + Trans;//平移，缩放

                face_vertices[j] = vert;
                face_textures[j] = Vector2f(mesh.Vertices[i + j].TextureCoordinate.X,
                                            mesh.Vertices[i + j].TextureCoordinate.Y);
                vertices_normal[j] = Vector3f(mesh.Vertices[i + j].Normal.X,
                                            mesh.Vertices[i + j].Normal.Y,
                                            mesh.Vertices[i + j].Normal.Z);
                // TODO: 在非均匀缩放下应当乘以法线矩阵

                min_vert = Vector3f(std::min(min_vert.x, vert.x),
                                    std::min(min_vert.y, vert.y),
                                    std::min(min_vert.z, vert.z));
                max_vert = Vector3f(std::max(max_vert.x, vert.x),
                                    std::max(max_vert.y, vert.y),
                                    std::max(max_vert.z, vert.z));
            }
            auto tempTriangle = Triangle(face_vertices[0], face_vertices[1],
                                   face_vertices[2], mt);
            tempTriangle.t0 = face_textures[0];
            tempTriangle.t1 = face_textures[1];
            tempTriangle.t2 = face_textures[2];
            tempTriangle.n0 = vertices_normal[0];
            tempTriangle.n1 = vertices_normal[1];
            tempTriangle.n2 = vertices_normal[2];
            
            triangles.emplace_back(tempTriangle);
            // triangles.emplace_back(face_vertices[0], face_vertices[1],
            //                        face_vertices[2], mt);
        }

        bounding_box = Bounds3(min_vert, max_vert); // 构建网格体的包围盒

        std::vector<Object*> ptrs;
        for (auto& tri : triangles){
            ptrs.push_back(&tri);
            area += tri.area;
        }
        bvh = new BVHAccel(ptrs); // 对一个网格体中所有三角形进行划分，构建BVH
    }

    Bounds3 getBounds() { return bounding_box; }

    // 光线与三角形网格体的交点，是由光线与自身的BVH树求交
    Intersection getIntersection(Ray ray)
    {
        Intersection intersec;

        if (bvh) {
            intersec = bvh->Intersect(ray);
        }

        return intersec;
    }
    
    // 对三角形网格体的采样，是对其BVH树的采样
    void Sample(Intersection &pos, float &pdf){
        bvh->Sample(pos, pdf);
        pos.emit = m->getEmission();
    }
    float getArea(){
        return area;
    }
    bool hasEmit(){
        return m->hasEmission();
    }
    
    // bool intersect(const Ray& ray) { return true; }

    // bool intersect(const Ray& ray, float& tnear, uint32_t& index) const
    // {
    //     bool intersect = false;
        

    //     for (uint32_t k = 0; k < numTriangles; ++k) {
    //         const Vector3f& v0 = vertices[vertexIndex[k * 3]];
    //         const Vector3f& v1 = vertices[vertexIndex[k * 3 + 1]];
    //         const Vector3f& v2 = vertices[vertexIndex[k * 3 + 2]];
    //         float t, u, v;
    //         if (rayTriangleIntersect(v0, v1, v2, ray.origin, ray.direction, t,
    //                                  u, v) &&
    //             t < tnear) {
    //             tnear = t;
    //             index = k;
    //             intersect |= true;
    //         }
    //     }

    //     return intersect;
    // }

    // void getSurfaceProperties(const Vector3f& P, const Vector3f& I,
    //                           const uint32_t& index, const Vector2f& uv,
    //                           Vector3f& N, Vector2f& st) const
    // {
    //     const Vector3f& v0 = vertices[vertexIndex[index * 3]];
    //     const Vector3f& v1 = vertices[vertexIndex[index * 3 + 1]];
    //     const Vector3f& v2 = vertices[vertexIndex[index * 3 + 2]];
    //     Vector3f e0 = normalize(v1 - v0);
    //     Vector3f e1 = normalize(v2 - v1);
    //     N = normalize(crossProduct(e0, e1));
    //     const Vector2f& st0 = stCoordinates[vertexIndex[index * 3]];
    //     const Vector2f& st1 = stCoordinates[vertexIndex[index * 3 + 1]];
    //     const Vector2f& st2 = stCoordinates[vertexIndex[index * 3 + 2]];
    //     st = st0 * (1 - uv.x - uv.y) + st1 * uv.x + st2 * uv.y;
    // }

    // Vector3f evalDiffuseColor(const Vector2f& st) const
    // {
    //     float scale = 5;
    //     float pattern =
    //         (fmodf(st.x * scale, 1) > 0.5) ^ (fmodf(st.y * scale, 1) > 0.5);
    //     return lerp(Vector3f(0.815, 0.235, 0.031),
    //                 Vector3f(0.937, 0.937, 0.231), pattern);
    // }
};

// bool rayTriangleIntersect(const Vector3f& v0, const Vector3f& v1,
//                           const Vector3f& v2, const Vector3f& orig,
//                           const Vector3f& dir, float& tnear, float& u, float& v)
// {
//     Vector3f edge1 = v1 - v0;
//     Vector3f edge2 = v2 - v0;
//     Vector3f pvec = crossProduct(dir, edge2);
//     float det = dotProduct(edge1, pvec);
//     if (det == 0 || det < 0)
//         return false;

//     Vector3f tvec = orig - v0;
//     u = dotProduct(tvec, pvec);
//     if (u < 0 || u > det)
//         return false;

//     Vector3f qvec = crossProduct(tvec, edge1);
//     v = dotProduct(dir, qvec);
//     if (v < 0 || u + v > det)
//         return false;

//     float invDet = 1 / det;

//     tnear = dotProduct(edge2, qvec) * invDet;
//     u *= invDet;
//     v *= invDet;

//     return true;
// }