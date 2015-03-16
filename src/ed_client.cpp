#include "ed_wbc/ed_client.h"

#include "serialization.h"

namespace ed_wbc {



EdClient::EdClient() {}

void EdClient::initialize()
{
    client_.launchProbe("wbc_probe", "libwbc_probe.so");
}

bool EdClient::getWorld(CollisionObjectPtrMap &world)
{
    // We do not have a specific request (just want to get all entity shapes), so leave request empty
    tue::serialization::Archive req;
    tue::serialization::Archive res;

    world.clear();

    // Ask the probe to process (in this case, retreive all entity shapes)
    if (client_.process(req, res))
    {
        serialization::deserializeCollisionWorld(res, world);
        return true;
    }
    else
    {
        return false; // processing failed
    }
}



} // namespace
