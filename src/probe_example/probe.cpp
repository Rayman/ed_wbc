#include "probe.h"
#include <ed/world_model.h>
#include <ed/entity.h>
#include <geolib/Shape.h>
#include <streambuf>

// ----------------------------------------------------------------------------------------------------

ExampleProbe::ExampleProbe()
{
}

// ----------------------------------------------------------------------------------------------------

ExampleProbe::~ExampleProbe()
{
}

// ----------------------------------------------------------------------------------------------------

void ExampleProbe::configure(tue::Configuration config)
{
}

// ----------------------------------------------------------------------------------------------------

void ExampleProbe::process(const ed::WorldModel& world,
             ed::UpdateRequest& update,
             tue::serialization::InputArchive& req,
             tue::serialization::OutputArchive& res)
{
    int i1, i2;
    req >> i1;
    req >> i2;

    std::stringstream shape_stream;

    int num_shapes = 0;

//    for(ed::WorldModel::const_iterator it = world.begin(); it != world.end(); it++)
//    {
//        ed::EntityConstPtr e = it->second;
//        geo::ShapeConstPtr shape = e->shape();

//        std::cout << it->first << std::endl;

//        if (shape) {
//            shape->write(shape_stream);
//            std::cout << "sent shape" << std::endl;
//            num_shapes++;
//        }
//    }

//    res << num_shapes << shape_stream.str();

    std::vector<geo::ShapeConstPtr> shapes;
    for(ed::WorldModel::const_iterator it = world.begin(); it != world.end(); it++)
    {
        ed::EntityConstPtr e = it->second;
        geo::ShapeConstPtr shape = e->shape();

        std::cout << it->first << std::endl;

        if (shape) {
            shapes.push_back(shape);
        }
    }

    res << (int)shapes.size();
    for(std::vector<geo::ShapeConstPtr>::const_iterator it = shapes.begin(); it != shapes.end(); ++it)
    {
        (*it)->write(res.getStream());
    }

}

ED_REGISTER_PLUGIN(ExampleProbe)
