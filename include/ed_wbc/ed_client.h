#ifndef EDCLIENT_H
#define EDCLIENT_H

#include <fcl/broadphase/broadphase.h>

namespace ed_wbc {



class EdClient
{
public:
    EdClient();

    fcl::BroadPhaseCollisionManager* getWorld();
};



} // namespace

#endif // EDCLIENT_H
