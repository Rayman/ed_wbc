#include "ed_wbc/ed_client.h"

#include <fcl/broadphase/broadphase_dynamic_AABB_tree.h>

namespace ed_wbc {



EdClient::EdClient()
{
}

fcl::BroadPhaseCollisionManager* getWorld()
{
    fcl::DynamicAABBTreeCollisionManager *manager = new fcl::DynamicAABBTreeCollisionManager();
    return manager;
}



} // namespace
