#include "wbc_probe.h"
#include <ed/world_model.h>
#include <ed/entity.h>

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

        // serialize vertices
        const std::vector<geo::Vector3>& vertices = shape->getMesh().getPoints();
        res << (int)vertices.size();
        for(unsigned int i = 0; i < vertices.size(); ++i)
        {
            const geo::Vector3& v = vertices[i];

            // Explicitly cast to float (4 byte) such that we know on the receiving end that we will get floats
            res << (float)v.x << (float)v.y << (float)v.z;
        }

        // serialize triangles
        const std::vector<geo::TriangleI>& triangles = shape->getMesh().getTriangleIs();
        res << (int)triangles.size();
        for(unsigned int i = 0; i < triangles.size(); ++i)
        {
            const geo::TriangleI& t = triangles[i];
            res << (float)t.i1_ << (float)t.i2_ << (float)t.i3_;
        }
    }
}

// Make sure ED can find this probe within the library
ED_REGISTER_PLUGIN(ShapeProbe)
