#ifndef EDWORLD_H
#define EDWORLD_H

#include "amigo_whole_body_controller/worldclient.h"

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

    void start();

    World* getWorld();

  protected:

    ros::Rate rate_;

    void loop();

    void update();

    boost::thread thread_;

    boost::mutex mutex_;

    boost::shared_ptr<fcl::BroadPhaseCollisionManager> world_;

    ed_wbc::EdClient client_;

};

} // namespace

#endif // EDWORLD_H
