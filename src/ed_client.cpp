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
        using namespace ed_wbc;

        std::vector< boost::shared_ptr<fcl::CollisionObject> > world;
        serialization::deserializeCollisionWorld(res, world);
        ROS_INFO("got %lu objects", world.size());
    }
    else
    {
        std::cout << "Probe processing failed." << std::endl;
    }

    return manager;
}



} // namespace
