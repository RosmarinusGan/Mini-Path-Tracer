#include "Renderer.hpp"
#include "Scene.hpp"
#include "Triangle.hpp"
#include "MeshTriangle.hpp"
#include "Sphere.hpp"
#include "Vector.hpp"
#include "global.hpp"
#include "Microfacet.hpp"
#include "Diffuse.hpp"
#include "Mirror.hpp"
#include "Transparent.hpp"
#include <chrono>

// TODO: 1.微表面模型（kulla-conty 参考202作业4），加入纹理（线性叠加，参考101作业3）（√，还有问题），以及复杂表面的细节效果 
// 2. 伽马矫正(√) 
// 3.体积散射(Volumetric path tracing) 重点！
// 4.多重重要性采样（针对微表面） 5.SAH加速,（次表面散射bssrdf）麻烦
// 漫反射采样似乎有问题？改进采样方法（√）
// 三角形法线好像并不是插值的，对于cornell盒来说无所谓，但是对于复杂模型有问题，需要改进(√)

// In the main function of the program, we create the scene (create objects and
// lights) as well as set the options for the render (image width and height,
// maximum recursion depth, field-of-view, etc.). We then call the render
// function().
int main(int argc, char** argv)
{
    // Change the definition here to change resolution
    Scene scene(1024, 1024);

    std::unique_ptr<Material> red = std::make_unique<Diffuse>(Vector3f(0.63f, 0.065f, 0.05f));
    red->Ks = Vector3f(0.7937, 0.7937, 0.7937);
    red->specularExponent = 50.0f;
    std::unique_ptr<Material> green = std::make_unique<Diffuse>(Vector3f(0.14f, 0.45f, 0.091f));
    green->Ks = Vector3f(0.7937, 0.7937, 0.7937);
    green->specularExponent = 50.0f;
    std::unique_ptr<Material> white = std::make_unique<Diffuse>(Vector3f(0.725f, 0.71f, 0.68f));
    white->Ks = Vector3f(0.7937, 0.7937, 0.7937);
    white->specularExponent = 50.0f;

    std::unique_ptr<Material> light = std::make_unique<Diffuse>(Vector3f(0.65f), (8.0f * Vector3f(0.747f+0.058f, 0.747f+0.258f, 0.747f) + 15.6f * Vector3f(0.740f+0.287f,0.740f+0.160f,0.740f) + 18.4f *Vector3f(0.737f+0.642f,0.737f+0.159f,0.737f)));
    
    std::unique_ptr<Material> glass = std::make_unique<Mirror>(30.0f); // 完美镜面反射
    // Material* glass_classic = new Material(MIRROR);
    // glass_classic->ior = 30.0f;

    std::unique_ptr<Material> green_m = std::make_unique<Microfacet>(Vector3f(0.14f, 0.45f, 0.091f));
    std::unique_ptr<Material> red_m = std::make_unique<Microfacet>(Vector3f(0.63f, 0.065f, 0.05f));

    std::unique_ptr<Material> diffuse = std::make_unique<Microfacet>();
    diffuse->ior = 12.85f;
    std::unique_ptr<Material> glossy = std::make_unique<Microfacet>(Vector3f(0.725f, 0.71f, 0.68f), 0.7f);
    glossy->ior = 12.85f;

    std::unique_ptr<Material> mirror = std::make_unique<Microfacet>(Vector3f(1.0f), 0.001f); // 还是不能取0，对完美镜面的采样似乎还是有问题
    std::unique_ptr<Material> diffuse_cow = std::make_unique<Microfacet>();
    diffuse_cow->setTexture("../models/spot/spot_texture.png");
    //diffuse_cow->setTexture("../models/rock/rock.png");
    //diffuse_cow->setTexture("../models/Crate/crate_1.jpg");
    diffuse_cow->ior = 12.85f;

    std::unique_ptr<Material> emission = std::make_unique<Diffuse>(Vector3f(1.f), Vector3f(63.f, 65.f, 50.f));

    // only for Whitted-Sytle
    std::unique_ptr<Material> transparent = std::make_unique<Transparent>(10.f, Vector3f(0.7937, 0.7937, 0.7937)); // ior 不能等于1,否则进入死循环运算

    MeshTriangle floor("../models/cornellbox/floor.obj", white.get());
    MeshTriangle shortbox("../models/cornellbox/shortbox.obj", diffuse.get());
    //MeshTriangle shortbox("../models/cornellbox/shortbox.obj", white, Vector3f(260, 0, 0), Vector3f(0.7f, 0.7f, 0.7f));
    //MeshTriangle tallbox("../models/cornellbox/tallbox.obj", glossy.get());
    MeshTriangle tallbox("../models/cornellbox/tallbox.obj", glossy.get());
    //MeshTriangle tallbox("../models/cornellbox/tallbox.obj", glass.get());
    MeshTriangle left("../models/cornellbox/left.obj", red.get());
    MeshTriangle right("../models/cornellbox/right.obj", green.get());

    MeshTriangle light_("../models/cornellbox/light.obj", light.get());

    MeshTriangle bunny("../models/bunny/bunny.obj", mirror.get(), Vector3f(130, -60, 150), 
        Vector3f(1500, 1500, 1500), Vector3f(-1, 0, 0), Vector3f(0, 1, 0), Vector3f(0, 0, -1));
    MeshTriangle teapot("../models/teapot.obj", red.get(), Vector3f(200, 100, -50), 
        Vector3f(55, 55, 55));
    MeshTriangle sphere1("../models/sphere.obj", white.get(), Vector3f(130, 100, 100), Vector3f(80.f));

    //MeshTriangle sphere2("../models/sphere.obj", mirror.get(), Vector3f(380, 100, 350), Vector3f(80.f));
    MeshTriangle sphere2("../models/sphere.obj", transparent.get(), Vector3f(380, 100, 350), Vector3f(80.f));

    MeshTriangle cow("../models/spot/spot_triangulated_good.obj", diffuse_cow.get(), Vector3f(300, 250, 200), Vector3f(50.f));
    MeshTriangle rock("../models/rock/rock.obj", diffuse_cow.get(), Vector3f(300, 250, 200), Vector3f(50.f));
    MeshTriangle crate("../models/Crate/Crate1.obj", diffuse_cow.get(), Vector3f(300, 300, 50), Vector3f(70.f));

    MeshTriangle sphereLight("../models/sphere.obj", light.get(), Vector3f(300.f, 200.f, -800.f), Vector3f(10.f));

    scene.Add(&floor);
    scene.Add(&shortbox);
    scene.Add(&tallbox);
    scene.Add(&left);
    scene.Add(&right);
    scene.Add(&light_);
    //scene.Add(&bunny);
    //scene.Add(&teapot);
    //scene.Add(&sphere1);
    //scene.Add(&sphere2);
    //scene.Add(&cow);
    //scene.Add(&rock);
    //scene.Add(&crate);
    //scene.Add(&sphereLight);

    scene.buildBVH();

    Renderer r;

    auto start = std::chrono::system_clock::now();
    r.Render(scene);
    auto stop = std::chrono::system_clock::now();

    std::cout << "Render complete: \n";
    std::cout << "Time taken: " << std::chrono::duration_cast<std::chrono::hours>(stop - start).count() << " hours\n";
    std::cout << "          : " << std::chrono::duration_cast<std::chrono::minutes>(stop - start).count() << " minutes\n";
    std::cout << "          : " << std::chrono::duration_cast<std::chrono::seconds>(stop - start).count() << " seconds\n";

    return 0;
}