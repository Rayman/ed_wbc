#include <amigo_whole_body_controller/wbc_node.h>
#include "edworld.h"

int main(int argc, char **argv) {

    ros::init(argc, argv, "whole_body_controller");
    ros::NodeHandle nh;

    ros::Rate loop_rate(50);
    wbc::WholeBodyControllerNode wbcEdNode(loop_rate);

    wbc::EdWorld world;
	wbcEdNode.setCollisionWorld(&world);

    StatsPublisher sp;
    sp.initialize();

    while (ros::ok()) {
        ros::spinOnce();

        sp.startTimer("main");
        wbcEdNode.update();
        sp.stopTimer("main");
        sp.publish();

        loop_rate.sleep();
    }

    return 0;
}
