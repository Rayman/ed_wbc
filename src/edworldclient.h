#ifndef EDWORLD_CLIENT_H
#define EDWORLD_CLIENT_H

#include "amigo_whole_body_controller/worldclient.h"
#include "edworld.h"

#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>

#include <ed_wbc/ed_client.h>
#include <ros/rate.h>
#include <ros/node_handle.h>

namespace wbc {

class EdWorldClient : public WorldClient
{
  public:

    EdWorldClient();

    void initialize();

    /** starts the world model update loop in a thread **/
    void start();

    /** get a fast copy of the current state of the world **/
    boost::shared_ptr<World> getWorld();

    void setIgnoredEntities(std::vector<std::string> entities);

  protected:

    ros::Rate rate_;

    /** runs update() with rate_ hz **/
    void loop();

    /** asks ed for a world model update **/
    void update();

    boost::thread thread_;

    boost::mutex mutex_;

    std::vector<std::string> ignored_entities;

    /** cached state of the world **/
    boost::shared_ptr< EdWorld > world_;

    /** connection to ed **/
    ed_wbc::EdClient client_;

    /** visualization of the collision world **/
    ros::NodeHandle private_nh_;
    ros::Publisher marker_pub;
};

} // namespace

#endif // EDWORLD_CLIENT_H
