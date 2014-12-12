#include "edworld.h"

namespace wbc {

EdWorld::EdWorld(std::vector< boost::shared_ptr<fcl::CollisionObject> > objects)
    : objects_(objects)
{
    for (std::vector< boost::shared_ptr<fcl::CollisionObject> >::iterator it = objects.begin(); it != objects.end(); ++it) {
        manager_.registerObject(it->get());
    }

    manager_.setup();
}

fcl::BroadPhaseCollisionManager* EdWorld::getCollisionManager()
{
    return &manager_;
}

} // namespace
