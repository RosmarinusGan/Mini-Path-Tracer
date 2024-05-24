#include <algorithm>
#include <cassert>
#include "BVH.hpp"

BVHAccel::BVHAccel(std::vector<Object*> p, int maxPrimsInNode,
                   SplitMethod splitMethod)
    : maxPrimsInNode(std::min(255, maxPrimsInNode)), splitMethod(splitMethod),
      primitives(std::move(p))
{
    time_t start, stop;
    time(&start);
    if (primitives.empty())
        return;

    root = recursiveBuild(primitives); // 生成BVH二叉树

    time(&stop);
    double diff = difftime(stop, start);
    int hrs = (int)diff / 3600;
    int mins = ((int)diff / 60) - (hrs * 60);
    int secs = (int)diff - (hrs * 3600) - (mins * 60);

    printf(
        "\rBVH Generation complete: \nTime Taken: %i hrs, %i mins, %i secs\n\n",
        hrs, mins, secs);
}

BVHBuildNode* BVHAccel::recursiveBuild(std::vector<Object*> objects)
{
    BVHBuildNode* node = new BVHBuildNode();

    // Compute bounds of all primitives in BVH node
    // Bounds3 bounds;
    // for (int i = 0; i < objects.size(); ++i)
    //     bounds = Union(bounds, objects[i]->getBounds());

    // 划分到结点中只有一个物体时结束
    // 理论上来说应该按照maxPrimsInNode来划分，但是这里为了方便，直接划分到只有一个物体
    if (objects.size() == 1) {
        // Create leaf _BVHBuildNode_
        node->bounds = objects[0]->getBounds();
        node->object = objects[0];
        node->left = nullptr;
        node->right = nullptr;
        node->area = objects[0]->getArea();
        return node;
    }
    else if (objects.size() == 2) {
        node->left = recursiveBuild(std::vector{objects[0]});
        node->right = recursiveBuild(std::vector{objects[1]});

        node->bounds = Union(node->left->bounds, node->right->bounds);
        node->area = node->left->area + node->right->area;
        return node;
    }
    else {
        Bounds3 centroidBounds;
        for (int i = 0; i < objects.size(); ++i)
            centroidBounds =
                Union(centroidBounds, objects[i]->getBounds().Centroid());
        int dim = centroidBounds.maxExtent();
        // switch (dim) {
        // case 0:
        //     std::sort(objects.begin(), objects.end(), [](auto& f1, auto& f2) {
        //         return f1->getBounds().Centroid().x <
        //                f2->getBounds().Centroid().x;
        //     });
        //     break;
        // case 1:
        //     std::sort(objects.begin(), objects.end(), [](auto& f1, auto& f2) {
        //         return f1->getBounds().Centroid().y <
        //                f2->getBounds().Centroid().y;
        //     });
        //     break;
        // case 2:
        //     std::sort(objects.begin(), objects.end(), [](auto& f1, auto& f2) {
        //         return f1->getBounds().Centroid().z <
        //                f2->getBounds().Centroid().z;
        //     });
        //     break;
        // }
        // auto middling = objects.begin() + objects.size() / 2;
        auto middling = quickSelect(objects, 0, objects.size() - 1, objects.size() / 2, dim); // 选取中位数
        auto beginning = objects.begin();
        auto ending = objects.end();

        auto leftshapes = std::vector<Object*>(beginning, middling);
        auto rightshapes = std::vector<Object*>(middling, ending);

        assert(objects.size() == (leftshapes.size() + rightshapes.size()));

        node->left = recursiveBuild(leftshapes);
        node->right = recursiveBuild(rightshapes);

        node->bounds = Union(node->left->bounds, node->right->bounds);
        node->area = node->left->area + node->right->area;
    }

    return node;
}

Intersection BVHAccel::Intersect(const Ray& ray) const
{
    Intersection isect;
    if (!root)
        return isect;
    isect = BVHAccel::getIntersection(root, ray); // 输入BVH二叉树和光线，返回交点
    return isect;
}

Intersection BVHAccel::getIntersection(BVHBuildNode* node, const Ray& ray) const
{
    // TODO Traverse the BVH to find intersection
    Intersection left, right, result;
    std::array<int, 3> dirIsNeg{ray.direction.x > 0, ray.direction.y > 0, ray.direction.z > 0};
    if(node->bounds.IntersectP(ray, ray.direction_inv, dirIsNeg)){
        if(node->left == nullptr && node->right == nullptr){
            return node->object->getIntersection(ray);
        }

        left = getIntersection(node->left, ray);
        right = getIntersection(node->right, ray);

        result = left.distance < right.distance ? left : right;
        return result;
    }else return result;
}

// 依据随机数p，node包围的所有物体中随机选取一个物体，并在这个物体上随机采样一点
void BVHAccel::getSample(BVHBuildNode* node, float p, Intersection &pos, float &pdf){
    if(node->left == nullptr || node->right == nullptr){
        node->object->Sample(pos, pdf);
        pdf *= node->area;
        return;
    }
    if(p < node->left->area) getSample(node->left, p, pos, pdf);
    else getSample(node->right, p - node->left->area, pos, pdf);
}

// 对bvh包围的所有物体中随机选取一个物体，并在这个物体上随机采样一点
void BVHAccel::Sample(Intersection &pos, float &pdf){
    float p = std::sqrt(get_random_float()) * root->area;
    getSample(root, p, pos, pdf);
    pdf /= root->area;
}

inline std::vector<Object *>::iterator quickSelect(std::vector<Object *>& objects, int start, int end, int k, int dim){
    if(start == end) return objects.begin() + start;
    int swapPos = start;
    int randomIndex = std::rand() % (end - start + 1) + start;
    float pivot;
    switch (dim)
    {
    case 0:
        pivot = objects[randomIndex]->getBounds().Centroid().x;
        for(int i = start; i <= end; i++){
            if(objects[i]->getBounds().Centroid().x <= pivot){
                std::swap(objects[swapPos], objects[i]);
                swapPos++;
            }
        }
        break;
    case 1:
        pivot = objects[randomIndex]->getBounds().Centroid().y;
        for(int i = start; i <= end; i++){
            if(objects[i]->getBounds().Centroid().y <= pivot){
                std::swap(objects[swapPos], objects[i]);
                swapPos++;
            }
        }
        break;
    case 2:
        pivot = objects[randomIndex]->getBounds().Centroid().z;
        for(int i = start; i <= end; i++){
            if(objects[i]->getBounds().Centroid().z <= pivot){
                std::swap(objects[swapPos], objects[i]);
                swapPos++;
            }
        }
        break;
    }
    if(swapPos - 1 == k)
        return objects.begin() + swapPos - 1;
    else if (swapPos - 1 > k)
        return quickSelect(objects, start, swapPos - 2, k, dim);
    else
        return quickSelect(objects, swapPos, end, k, dim);
}