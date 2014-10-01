#ifndef EDWORLD_CLIENT_H
#define EDWORLD_CLIENT_H

#include "amigo_whole_body_controller/worldclient.h"
#include "edworld.h"

#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>

#include <ed_wbc/ed_client.h>
#include <ros/rate.h>

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

  protected:

    ros::Rate rate_;

    /** runs update() with rate_ hz **/
    void loop();

    /** asks ed for a world model update **/
    void update();

    boost::thread thread_;

    boost::mutex mutex_;

    /** cached state of the world **/
    boost::shared_ptr< EdWorld > world_;

    /** connection to ed **/
    ed_wbc::EdClient client_;

};

} // namespace

#endif // EDWORLD_CLIENT_H
