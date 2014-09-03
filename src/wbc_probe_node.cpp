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
            using namespace ed_wbc;

            serialization::deserializeCollisionWorld(res);
        }
        else
        {
            std::cout << "Probe processing failed." << std::endl;
        }
    }

    return 0;
}
