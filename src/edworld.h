#ifndef EDWORLD_H
#define EDWORLD_H

#include "amigo_whole_body_controller/world.h"
#include <fcl/broadphase/broadphase_dynamic_AABB_tree.h>

namespace wbc {

class EdWorld : public World
{
  public:
    EdWorld();

    EdWorld(std::vector< boost::shared_ptr<fcl::CollisionObject> > objects);

    fcl::BroadPhaseCollisionManager* getCollisionManager();

  protected:

    std::vector< boost::shared_ptr<fcl::CollisionObject> > objects_;

    fcl::DynamicAABBTreeCollisionManager manager_;

};

} // namespace

#endif // EDWORLD_H
