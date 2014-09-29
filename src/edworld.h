#ifndef EDWORLD_H
#define EDWORLD_H

#include "amigo_whole_body_controller/world.h"
#include <fcl/broadphase/broadphase_dynamic_AABB_tree.h>

namespace wbc {

class EdWorld : public World
{
  public:
    EdWorld();

    /**
     * @brief Initialize world with a set of collision objects
     *
     * A collision manager contains pointer's to CollisionObjects.
     * This means that a manager is not self contained. This class however,
     * is self contained. It creates a copy of the objects vector. Each element
     * is a boost::shared_ptr to an object.
     */
    EdWorld(std::vector< boost::shared_ptr<fcl::CollisionObject> > objects);

    fcl::BroadPhaseCollisionManager* getCollisionManager();

  protected:

    std::vector< boost::shared_ptr<fcl::CollisionObject> > objects_;

    fcl::DynamicAABBTreeCollisionManager manager_;

};

} // namespace

#endif // EDWORLD_H
