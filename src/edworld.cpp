#include "edworld.h"

namespace wbc {

EdWorld::EdWorld(std::vector< boost::shared_ptr<fcl::CollisionObject> > objects)
    : objects_(objects)
{
    // convert to an array of pointers so it can be fed to registerObjects
    std::vector<fcl::CollisionObject *> object_ptrs;
    for (std::vector< boost::shared_ptr<fcl::CollisionObject> >::iterator it = objects.begin(); it != objects.end(); ++it) {
        object_ptrs.push_back(it->get());
    }

    manager_.registerObjects(object_ptrs);

    // don't call update() here or else the BVH will be broken
    //   - Ramon (fcl bug?)
}

fcl::BroadPhaseCollisionManager* EdWorld::getCollisionManager()
{
    return &manager_;
}

} // namespace
