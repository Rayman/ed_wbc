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

World* EdWorldClient::getWorld()
{
    boost::lock_guard<boost::mutex> lock(mutex_);
    return 0; // todo return a world
}

void EdWorldClient::update()
{
    fcl::BroadPhaseCollisionManager *world = client_.getWorld();

    if (!world) {
        ROS_ERROR("probe processing failed");
        return;
    }

    ROS_INFO("ed world update: %lu entities", world->size());
    boost::shared_ptr<fcl::BroadPhaseCollisionManager> world_ptr(world);

    boost::lock_guard<boost::mutex> lock(mutex_);
    world_ = world_ptr;
}

} // namespace
