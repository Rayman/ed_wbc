#include "wbc_probe.h"
#include <ed/world_model.h>
#include <ed/entity.h>

#include "serialization.h"
#include <geolib/Shape.h>

// ----------------------------------------------------------------------------------------------------

ShapeProbe::ShapeProbe()
{
}

// ----------------------------------------------------------------------------------------------------

ShapeProbe::~ShapeProbe()
{
}

// ----------------------------------------------------------------------------------------------------

void ShapeProbe::configure(tue::Configuration config)
{
}

// ----------------------------------------------------------------------------------------------------

void ShapeProbe::process(const ed::WorldModel& world,
             ed::UpdateRequest& update,
             tue::serialization::InputArchive& req,
             tue::serialization::OutputArchive& res)
{
    using namespace ed_wbc;

    // Loop over all world entities and find entities that have a shape
    std::vector<ed::EntityConstPtr> shape_entities;
    for(ed::WorldModel::const_iterator it = world.begin(); it != world.end(); ++it)
    {
        const ed::EntityConstPtr& e = it->second;
        if (e->shape())
            shape_entities.push_back(e);
    }

    // Add the number of entities to the response
    res << (int)shape_entities.size();
    for(std::vector<ed::EntityConstPtr>::const_iterator it = shape_entities.begin(); it != shape_entities.end(); ++it)
    {
        const ed::EntityConstPtr& e = *it;
        geo::ShapeConstPtr shape = e->shape();

        // Add the id of the entity to the response
        res << e->id();

        serialization::serialize(*shape, res.stream());
    }
}

// Make sure ED can find this probe within the library
ED_REGISTER_PLUGIN(ShapeProbe)
