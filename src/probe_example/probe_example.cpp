#include <ed/io/transport/probe_client.h>
#include <sstream>

#include <geolib/Shape.h>

int main(int argc, char **argv) {

    ed::ProbeClient client;
    client.launchProbe("example_probe", "/home/ramon/dev_ws/devel/lib/libed_example_probe.so");

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
            int total;
            res >> total;
            std::cout << "Total of " << total << " shapes." << std::endl;

            geo::ShapePtr shape = geo::Shape::read(res.stream());


        }
        else
        {
            std::cout << "Probe processing failed." << std::endl;
        }
    }

    return 0;
}
