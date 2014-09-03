#include "serialization.h"

#include <geolib/Shape.h>

namespace ed_wbc {



namespace serialization {

bool serialize(const geo::Shape& shape, std::ostream& output)
{
    // serialize vertices
    const std::vector<geo::Vector3>& vertices = shape.getMesh().getPoints();
    output << (int)vertices.size();
    for(unsigned int i = 0; i < vertices.size(); ++i)
    {
        const geo::Vector3& v = vertices[i];

        // Explicitly cast to float (4 byte) such that we know on the receiving end that we will get floats
        output << (float)v.x << (float)v.y << (float)v.z;
    }

    // serialize triangles
    const std::vector<geo::TriangleI>& triangles = shape.getMesh().getTriangleIs();
    output << (int)triangles.size();
    for(unsigned int i = 0; i < triangles.size(); ++i)
    {
        const geo::TriangleI& t = triangles[i];
        output << (int)t.i1_ << (int)t.i2_ << (int)t.i3_;
    }
}

const boost::shared_ptr<fcl::CollisionObject> deserialize(std::istream& input)
{

}

}



} // namespace
