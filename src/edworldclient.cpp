#include "edworldclient.h"

#include <fcl/broadphase/broadphase_dynamic_AABB_tree.h>
#include <ros/init.h>
#include <ros/console.h>

namespace wbc {

EdWorldClient::EdWorldClient() :
    rate_(5)
{

}

void EdWorldClient::initialize()
{
    client_.initialize();
    update();
}

void EdWorldClient::loop()
{
    while (ros::ok())
    {
        update();
        rate_.sleep();
    }
}

void EdWorldClient::start()
{
    thread_ = boost::thread(&EdWorldClient::loop, this);
}

boost::shared_ptr<World> EdWorldClient::getWorld()
{
    boost::lock_guard<boost::mutex> lock(mutex_);
    return world_;
}

void EdWorldClient::update()
{
    std::vector< ed_wbc::CollisionObjectPtr > world;
    if (!client_.getWorld(world)) {
        ROS_ERROR("probe processing failed");
        return;
    }

    ROS_INFO("ed world update: %lu entities", world.size());

    // TODO: create a new world object

    boost::lock_guard<boost::mutex> lock(mutex_);
    //world_ = world_ptr;
}

} // namespace
