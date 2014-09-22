#ifndef EDCLIENT_H
#define EDCLIENT_H

#include <fcl/broadphase/broadphase.h>
#include <ed/io/transport/probe_client.h>

namespace ed_wbc {



class EdClient
{
    ed::ProbeClient client_;

public:
    EdClient();

    void initialize();

    fcl::BroadPhaseCollisionManager* getWorld();
};



} // namespace

#endif // EDCLIENT_H
