#include <ed/io/transport/probe_client.h>

int main(int argc, char **argv) {

    ed::ProbeClient client;
    client.launchProbe("example_shape_probe", "libed_example_shape_probe.so");

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

                // - get the number of vertices, and content of these vertices
                int num_vertices;
                res >> num_vertices;
                for(int j = 0; j < num_vertices; ++j)
                {
                    float x, y, z;
                    res >> x >> y >> z;
                }

                // - get the number of triangles, and content of these triangles
                int num_triangles;
                res >> num_triangles;
                for(int j = 0; j < num_triangles; ++j)
                {
                    int i1, i2, i3;
                    res >> i1 >> i2 >> i3;
                }

                // Do something fancy with the obtained mesh
                // ...

                // ... or simply print the number of vertices and triangles:
                std::cout << "      " << num_vertices << " vertices" << std::endl;
                std::cout << "      " << num_triangles << " triangles" << std::endl;

            }
        }
        else
        {
            std::cout << "Probe processing failed." << std::endl;
        }
    }

    return 0;
}
