#include <ed/io/transport/probe_client.h>

#include "serialization.h"

int main(int argc, char **argv) {

    ed::ProbeClient client;
    client.launchProbe("wbc_probe", "libwbc_probe.so");

    // Just for the fun of it, let the probe process 10 times
    for(int i_test = 0; i_test < 10; ++i_test)
    {
        // We do not have a specific request (just want to get all entity shapes), so leave request empty
        tue::serialization::Archive req;
        tue::serialization::Archive res;

        // Ask the probe to process (in this case, retreive all entity shapes)
        if (client.process(req, res))
        {
            // Get the number of shapes
            int num_shapes;
            res >> num_shapes;

            std::cout << num_shapes << " shapes" << std::endl;
            for(int i = 0; i < num_shapes; ++i)
            {
                // For each shape:
                // - get the entitiy id
                std::string entity_id;
                res >> entity_id;

                std::cout << "  - " << entity_id << ":" << std::endl;

                boost::shared_ptr<fcl::CollisionObject> obj = ed_wbc::serialization::deserialize(res);
            }
        }
        else
        {
            std::cout << "Probe processing failed." << std::endl;
        }
    }

    return 0;
}
