#include <ed/io/transport/probe_client.h>
#include <sstream>

int main(int argc, char **argv) {

    ed::ProbeClient client;
    client.launchProbe("example_probe", "/home/sdries/ros/hydro/dev/devel/lib/libed_example_probe.so");

    for(int i = 0; i < 10; ++i)
    {

        int i1 = 5 + i;
        int i2 = 7;

        tue::serialization::Archive req;
        req << i1;
        req << i2;

        tue::serialization::Archive res;

        if (client.process(req, res))
        {
            int sum;
            res >> sum;

            std::cout << i1 << " + " << i2 << " = " << sum << std::endl;

            std::string message;
            res >> message;

            std::cout << message << std::endl;
        }
        else
        {
            std::cout << "Probe processing failed." << std::endl;
        }
    }

    return 0;
}
