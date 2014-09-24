#include "ed_wbc/ed_client.h"

#include "serialization.h"
#include <fcl/broadphase/broadphase_dynamic_AABB_tree.h>

namespace ed_wbc {



EdClient::EdClient() {}

void EdClient::initialize()
{
    client_.launchProbe("wbc_probe", "libwbc_probe.so");
}

fcl::BroadPhaseCollisionManager* EdClient::getWorld()
{
    fcl::DynamicAABBTreeCollisionManager *manager = new fcl::DynamicAABBTreeCollisionManager();

    // We do not have a specific request (just want to get all entity shapes), so leave request empty
    tue::serialization::Archive req;
    tue::serialization::Archive res;

    // Ask the probe to process (in this case, retreive all entity shapes)
    if (client_.process(req, res))
    {
        std::vector<serialization::WorldCollisionObject> world;
        serialization::deserializeCollisionWorld(res, world);
        for (std::vector<serialization::WorldCollisionObject>::iterator it = world.begin(); it != world.end(); ++it) {
            manager->registerObject(it->get());
        }
    }
    else
    {
        ROS_ERROR("probe processing failed");
        return 0;
    }

    manager->setup();

    return manager;
}



} // namespace
