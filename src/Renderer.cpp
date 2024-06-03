#include <fstream>
#include "Scene.hpp"
#include "Renderer.hpp"
#include "global.hpp"
#include "Vector.hpp"
#include <mingw.thread.h>
#include <mingw.mutex.h>

inline float deg2rad(const float& deg) { return deg * M_PI / 180.0; }

// epsilon值大小会影响结果的亮度，如果太小，会出现横状黑色条纹，原因是直接光部分的精度问题
const float EPSILON = 0.00016f;

// The main render function. This where we iterate over all pixels in the image,
// generate primary rays and cast these rays into the scene. The content of the
// framebuffer is saved to a file.
void Renderer::Render(const Scene& scene)
{
    std::vector<Vector3f> framebuffer(scene.width * scene.height);

    float scale = tan(deg2rad(scene.fov * 0.5));
    float imageAspectRatio = scene.width / (float)scene.height;
    Vector3f eye_pos(278, 273, -800);
    int m = 0;

    // change the spp value to change sample ammount
    int spp = 128; // 每个pixel路径数
    std::cout << "SPP: " << spp << "\n";

    int thread_num = 6; // 线程数
    int thread_height = scene.height / thread_num; // 每个线程处理的高度/行数
    std::vector<std::thread> threads(thread_num);
    std::mutex mtx;
    int progress = 0;

    bool isBasic = false; // 是否使用whitted-style ray tracing

    // 使用lamdba表达式定义函数对象，描述每个线程的任务
    auto renderEachRow = [&](int thread_index){
        int start = thread_index * thread_height;
        int end = (thread_index + 1) * thread_height;
        for (uint32_t j = start; j < end; ++j) {
            for (uint32_t i = 0; i < scene.width; ++i) {
                // generate primary ray direction
                // float x = (2 * (i + get_random_float()) / (float)scene.width - 1) *
                //         imageAspectRatio * scale;
                // float y = (1 - 2 * (j + get_random_float()) / (float)scene.height) * scale;

                // float x = (2 * (i + 0.5) / (float)scene.width - 1) *
                //         imageAspectRatio * scale;
                // float y = (1 - 2 * (j + 0.5) / (float)scene.height) * scale;

                //Vector3f dir = Vector3f(-x, y, 1).normalized();

                // MSAA抗锯齿
                int num = std::ceil(sqrt(spp));
                float invNum = 1.f / num;
                float invNumHalf = invNum * 0.5f;

                for (int k = 0; k < spp; k++){
                    float screen_i = i + invNumHalf + invNum * (k % num);
                    float screen_j = j + invNumHalf + invNum * (k / num);
                    // 从屏幕像素坐标转换到[-1，1]再转换为相机坐标系下的坐标
                    float x = ((2 * screen_i / (float)scene.width) - 1) *
                            imageAspectRatio * scale;
                    float y = (1 - (2 * screen_j / (float)scene.height)) * scale;
                    // 因为认为相机的始终朝向z轴，因此无论相机在哪个位置，dir都可以按照相机在原点计算，即在相机坐标系下计算（如果相机朝向不是这样，那dir可能要进行坐标系转换，从相机坐标系转换到世界坐标系）
                    Vector3f dir = normalize(Vector3f(-x, y, 1));

                    if(isBasic){
                        framebuffer[(int)(j * scene.width + i)] += scene.castRayBasic(Ray(eye_pos, dir)) / spp; // whitted-style tracing
                    }else{
                        framebuffer[(int)(j * scene.width + i)] += scene.castRayPT(Ray(eye_pos, dir)) / spp; // path tracing
                    }
                }
            }

            mtx.lock(); // 一行渲染完成后更新进度条
            progress++;
            UpdateProgress(progress / (float)scene.height);
            mtx.unlock();
        }
    };

    // 给线程分配任务
    for(int i = 0; i < thread_num; ++i){
        threads[i] = std::thread(renderEachRow, i);
    }
    for(int i = 0; i < thread_num; ++i){
        threads[i].join();
    }
    
    UpdateProgress(1.f);

    // save framebuffer to file
    FILE* fp = fopen("pathTracing.ppm", "wb");
    (void)fprintf(fp, "P6\n%d %d\n255\n", scene.width, scene.height);
    for (auto i = 0; i < scene.height * scene.width; ++i) {
        static unsigned char color[3];
        // gamma correction
        //framebuffer[i] = pow(framebuffer[i], 1 / GAMMA_C); 
        
        color[0] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].x), 0.6f));
        color[1] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].y), 0.6f));
        color[2] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].z), 0.6f));
        fwrite(color, 1, 3, fp);
    }
    fclose(fp);    
}
