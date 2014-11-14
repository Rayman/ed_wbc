#include "edworldclient.h"

#include <ros/init.h>
#include <ros/console.h>

#include <amigo_whole_body_controller/conversions.h>

namespace wbc {

EdWorldClient::EdWorldClient() :
    rate_(5), private_nh_("~")
{
    marker_pub = private_nh_.advertise<visualization_msgs::Marker>("ed_world", 1);
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
    std::vector< ed_wbc::CollisionObjectPtr > objects;
    if (!client_.getWorld(objects)) {
        ROS_ERROR("probe processing failed");
        return;
    }

    ROS_INFO("ed world update: %lu entities", objects.size());

    // visualize the collision world
    if (marker_pub.getNumSubscribers() > 0) {
        for (std::vector< ed_wbc::CollisionObjectPtr >::const_iterator it = objects.begin(); it != objects.end(); ++it) {
            visualization_msgs::Marker triangle_list;
            wbc::objectFCLtoMarker(**it, triangle_list);
            marker_pub.publish(triangle_list);
        }
    }

    EdWorld *world = new EdWorld(objects);

    // lock and swap pointers
    boost::lock_guard<boost::mutex> lock(mutex_);
    world_ = boost::shared_ptr<EdWorld>(world);
}

} // namespace
