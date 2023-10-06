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

    root = recursiveBuild(primitives, 0);

    time(&stop);
    double diff = difftime(stop, start);
    int hrs = (int)diff / 3600;
    int mins = ((int)diff / 60) - (hrs * 60);
    int secs = (int)diff - (hrs * 3600) - (mins * 60);

    printf(
        "\rBVH Generation complete: \nTime Taken: %i hrs, %i mins, %i secs\n\n",
        hrs, mins, secs);
}

BVHBuildNode* BVHAccel::recursiveBuild(std::vector<Object*> objects, int dim)
{
    BVHBuildNode* node = new BVHBuildNode();

    // Compute bounds of all primitives in BVH node
    Bounds3 bounds;
    for (int i = 0; i < objects.size(); ++i)
        bounds = Union(bounds, objects[i]->getBounds());
    if (objects.size() == 1) {
        // Create leaf _BVHBuildNode_
        node->bounds = objects[0]->getBounds();
        node->object = objects[0];
        node->left = nullptr;
        node->right = nullptr;
        return node;
    }
    else if (objects.size() == 2) {
        node->left = recursiveBuild(std::vector{objects[0]}, dim);
        node->right = recursiveBuild(std::vector{objects[1]}, dim);

        node->bounds = Union(node->left->bounds, node->right->bounds);
        return node;
    }
    else {

        // TODO: modify this function to make rendering faster, e.g. using k-d tree
        // hint: make use of dim variable

        // Updated Method
        // Time taken: 0 hours
        //   : 0 minutes
        //   : 6 ~ 8 seconds
        if(dim%3==0){ 
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                    return f1->getBounds().Centroid().x <
                        f2->getBounds().Centroid().x;
                });
        }else if(dim%3==1){
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                    return f1->getBounds().Centroid().y <
                        f2->getBounds().Centroid().y;
                });
        }else{
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                    return f1->getBounds().Centroid().z <
                        f2->getBounds().Centroid().z;
                });
        }
        dim += 1;

        // Original Method
        // Time taken: 0 hours
        //   : 0 minutes
        //   : 26 ~ 27 seconds
        // std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
        //         return f1->getBounds().Centroid().x <
        //             f2->getBounds().Centroid().x;
        //     });

        auto beginning = objects.begin();
        auto middling = objects.begin() + (objects.size() / 2);
        auto ending = objects.end();

        auto leftshapes = std::vector<Object*>(beginning, middling);
        auto rightshapes = std::vector<Object*>(middling, ending);

        assert(objects.size() == (leftshapes.size() + rightshapes.size()));

        node->left = recursiveBuild(leftshapes, dim);
        node->right = recursiveBuild(rightshapes, dim);

        node->bounds = Union(node->left->bounds, node->right->bounds);
    }

    return node;
}

Intersection BVHAccel::Intersect(const Ray& ray) const
{
    Intersection isect;
    if (!root)
        return isect;
    isect = BVHAccel::getIntersection(root, ray);
    return isect;
}

Intersection BVHAccel::getIntersection(BVHBuildNode* node, const Ray& ray) const
{


    Intersection null_inter; // null intersection by default

    Vector3f indiv(1.0f/ray.direction[0], 1.0f/ray.direction[1], 1.0f/ray.direction[2]);

    std::array<int, 3> dirIsNeg = { {int(ray.direction.x>0), int(ray.direction.y>0), int(ray.direction.z>0)}};


    // TODO: Traverse the BVH to find intersection. you might want to call Bounds3::IntersectP, 
    // or Object::getIntersection, or recursively BVHAccel::getIntersection
    
    if(node->bounds.IntersectP(ray, indiv, dirIsNeg)){
        if(node->left == nullptr && node->right == nullptr){
            return node->object->getIntersection(ray);
        }
        else{
            Intersection left = BVHAccel::getIntersection(node->left, ray);
            Intersection right = BVHAccel::getIntersection(node->right, ray);
            if(left.happened && right.happened){
                return left.distance < right.distance ? left : right;
            }
            else if(left.happened){
                return left;
            }
            else if(right.happened){
                return right;
            }
        }
    }
    return null_inter; // return null intersection in case ray does not intersect with bounding box
}