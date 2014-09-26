#ifndef EDCLIENT_H
#define EDCLIENT_H

#include <ed/io/transport/probe_client.h>
#include <fcl/collision_object.h>

namespace ed_wbc {

typedef boost::shared_ptr< fcl::CollisionObject > CollisionObjectPtr;

class EdClient
{
    ed::ProbeClient client_;

public:
    EdClient();

    void initialize();

    bool getWorld(std::vector< CollisionObjectPtr > &world);
};



} // namespace

#endif // EDCLIENT_H
