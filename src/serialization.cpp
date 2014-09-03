#include "serialization.h"

#include <geolib/Shape.h>
#include <fcl/BVH/BVH_model.h>

namespace ed_wbc {



namespace serialization {

bool serialize(const geo::Shape& shape, tue::serialization::OutputArchive& output)
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

const boost::shared_ptr<fcl::CollisionObject> deserialize(tue::serialization::Archive &input)
{
    std::vector<fcl::Vec3f>    vertices;
    std::vector<fcl::Triangle> triangles;

    // - get the number of vertices, and content of these vertices
    int num_vertices;
    input >> num_vertices;
    vertices.reserve(num_vertices);
    for(int j = 0; j < num_vertices; ++j)
    {
        float x, y, z;
        input >> x >> y >> z;
        vertices.push_back(fcl::Vec3f(x, y, z));
    }

    // - get the number of triangles, and content of these triangles
    int num_triangles;
    input >> num_triangles;
    triangles.reserve(num_triangles);
    for(int j = 0; j < num_triangles; ++j)
    {
        int p1, p2, p3;
        input >> p1 >> p2 >> p3;
        triangles.push_back(fcl::Triangle(p1, p2, p3));
    }

    fcl::BVHModel<fcl::OBBRSS>* model = new fcl::BVHModel<fcl::OBBRSS>();
    model->beginModel();
    model->addSubModel(vertices, triangles);
    model->endModel();

    //delete model;

    // ... or simply print the number of vertices and triangles:
    std::cout << "      " << num_vertices << " vertices" << std::endl;
    std::cout << "      " << num_triangles << " triangles" << std::endl;

    boost::shared_ptr<fcl::CollisionGeometry> geom(model);

    return boost::shared_ptr<fcl::CollisionObject>(new fcl::CollisionObject(geom));
}

}



} // namespace
