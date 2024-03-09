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

    root = recursiveBuild(primitives);

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
    bool useBVH = false;

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
        node->left = recursiveBuild(std::vector{objects[0]});
        node->right = recursiveBuild(std::vector{objects[1]});

        node->bounds = Union(node->left->bounds, node->right->bounds);
        return node;
    }
    else 
    {
        Bounds3 centroidBounds;
        for (int i = 0; i < objects.size(); ++i)
            centroidBounds =
                Union(centroidBounds, objects[i]->getBounds().Centroid());
        if (useBVH)
        {
            int dim = centroidBounds.maxExtent();
            switch (dim) {
            case 0:
                std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                    return f1->getBounds().Centroid().x <
                           f2->getBounds().Centroid().x;
                });
                break;
            case 1:
                std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                    return f1->getBounds().Centroid().y <
                           f2->getBounds().Centroid().y;
                });
                break;
            case 2:
                std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                    return f1->getBounds().Centroid().z <
                           f2->getBounds().Centroid().z;
                });
                break;
            }
            auto beginning = objects.begin();
            auto middling = objects.begin() + (objects.size() / 2);
            auto ending = objects.end();

            auto leftshapes = std::vector<Object*>(beginning, middling);
            auto rightshapes = std::vector<Object*>(middling, ending);

            assert(objects.size() == (leftshapes.size() + rightshapes.size()));

            node->left = recursiveBuild(leftshapes);
            node->right = recursiveBuild(rightshapes);

            node->bounds = Union(node->left->bounds, node->right->bounds);
        }
        else
        {
            int splitAxis = 0;
            int splitB = 0;
            float B = 8;
            double minCost = std::numeric_limits<double>::max();
            float divSN = 1 / bounds.SurfaceArea();
            for (int i = 0; i < 3; i++)
            {
                switch (i) 
                {
                case 0:
                    std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                        return f1->getBounds().Centroid().x < f2->getBounds().Centroid().x;
                        }); break;
                case 1:
                    std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                        return f1->getBounds().Centroid().y < f2->getBounds().Centroid().y;
                        }); break;
                case 2:
                    std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                        return f1->getBounds().Centroid().z < f2->getBounds().Centroid().z;
                        }); break;
                }
                double cost;
                for (int j = 1; j < B; j++)
                {
                    auto beginning = objects.begin();
                    auto middling = objects.begin() + (objects.size() * j / B);
                    auto ending = objects.end();

                    auto leftshapes = std::vector<Object* >(beginning, middling);
                    auto rightshapes = std::vector<Object* >(middling, ending);

                    assert(objects.size() == (leftshapes.size() + rightshapes.size()));

                    Bounds3 leftBounds, rightBounds;
                    for (int i = 0; i < leftshapes.size(); i++)
                        leftBounds = Union(leftBounds, leftshapes[i]->getBounds());
                    for (int i = 0; i < rightshapes.size(); i++)
                        rightBounds = Union(rightBounds, rightshapes[i]->getBounds());

                    cost = (leftBounds.SurfaceArea() * leftshapes.size() + rightBounds.SurfaceArea() * rightshapes.size());
                    if (cost < minCost)
                    {
                        minCost = cost;
                        splitAxis = i;
                        splitB = j;
                    }
                }
            }

            switch (splitAxis) 
            {
            case 0:
                std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                    return f1->getBounds().Centroid().x < f2->getBounds().Centroid().x;
                    }); break;
            case 1:
                std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                    return f1->getBounds().Centroid().y < f2->getBounds().Centroid().y;
                    }); break;
            case 2:
                std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                    return f1->getBounds().Centroid().z < f2->getBounds().Centroid().z;
                    }); break;
            }

            auto beginning = objects.begin();
            auto middling = objects.begin() + (objects.size() * splitB / B);
            auto ending = objects.end();

            auto leftshapes = std::vector<Object*>(beginning, middling);
            auto rightshapes = std::vector<Object*>(middling, ending);

            assert(objects.size() == (leftshapes.size() + rightshapes.size()));

            node->left = recursiveBuild(leftshapes);
            node->right = recursiveBuild(rightshapes);

            node->bounds = Union(node->left->bounds, node->right->bounds);
        }
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
    // TODO Traverse the BVH to find intersection
    Intersection result;
    if (node == nullptr || !node->bounds.IntersectP(ray, ray.direction_inv, {ray.direction.x > 0, ray.direction.y > 0, ray.direction.z > 0}))
        return result;
    if (node->left == nullptr && node->right == nullptr)
        return node->object->getIntersection(ray);
    Intersection hitLeft, hitRight;
    if (node->left != nullptr)
        hitLeft = getIntersection(node->left, ray);
    if (node->right != nullptr)
        hitRight = getIntersection(node->right, ray);
    result = hitLeft.distance < hitRight.distance ? hitLeft : hitRight;
    return result;
}