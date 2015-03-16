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

void EdWorldClient::setIgnoredEntities(std::vector<std::string> entities)
{
    boost::lock_guard<boost::mutex> lock(mutex_);
    ignored_entities = entities;
}

void EdWorldClient::update()
{
    std::vector< ed_wbc::CollisionObjectPtr > objects;
    ed_wbc::CollisionObjectPtrMap object_map;

    if (!client_.getWorld(object_map)) {
        ROS_ERROR("probe processing failed");
        return;
    }

    {
        boost::lock_guard<boost::mutex> lock(mutex_);
        for (ed_wbc::CollisionObjectPtrMap::iterator it = object_map.begin(); it != object_map.end(); ++it) {
            // if the entityId is not ignored, process it
            if (std::find(ignored_entities.begin(), ignored_entities.end(), it->first) == ignored_entities.end()) {
                objects.push_back(it->second);
            }
        }
    }

    ROS_DEBUG("ed world update: %lu entities", objects.size());

    // visualize the collision world
    if (marker_pub.getNumSubscribers() > 0) {
        int id = 0;
        for (std::vector< ed_wbc::CollisionObjectPtr >::const_iterator it = objects.begin(); it != objects.end(); ++it) {
            visualization_msgs::Marker m;
            wbc::objectFCLtoMarker(**it, m);

            m.id = ++id;
            m.header.frame_id = "/map";

            m.color.a = 0.5;
            m.color.r = 0;
            m.color.g = 0;
            m.color.b = 1;

            marker_pub.publish(m);
        }
    }

    EdWorld *world = new EdWorld(objects);

    // lock and swap pointers
    boost::lock_guard<boost::mutex> lock(mutex_);
    world_ = boost::shared_ptr<EdWorld>(world);
}

} // namespace
