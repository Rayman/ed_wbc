#include "serialization.h"

#include <geolib/Shape.h>
#include <ed/entity.h>
#include <fcl/BVH/BVH_model.h>

namespace ed_wbc {



namespace serialization {

void serialize(const geo::Shape& shape, tue::serialization::OutputArchive& output)
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

void serializeCollisionWorld(const ed::WorldModel& world, tue::serialization::OutputArchive& output)
{
    using namespace ed_wbc;

    // Loop over all world entities and find entities that have a shape
    std::vector<ed::EntityConstPtr> shape_entities;
    for(ed::WorldModel::const_iterator it = world.begin(); it != world.end(); ++it)
    {
        const ed::EntityConstPtr& e = it->second;

        ROS_INFO("entity %p", e.get());
        if (!e.get()) {
            ROS_WARN("NULL entity found");
            continue;
        }

        if (e->shape())
            shape_entities.push_back(e);
    }

    // Add the number of entities to the response
    ROS_INFO("serializing %i entities", (int)shape_entities.size());
    output << (int)shape_entities.size();
    for(std::vector<ed::EntityConstPtr>::const_iterator it = shape_entities.begin(); it != shape_entities.end(); ++it)
    {
        const ed::EntityConstPtr& e = *it;
        geo::ShapeConstPtr shape = e->shape();

        // Add the id of the entity to the response
        output << e->id();

        serialization::serialize(*shape, output);
    }
}

boost::shared_ptr<fcl::CollisionObject> deserialize(tue::serialization::Archive &input)
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

void deserializeCollisionWorld(tue::serialization::Archive &input, std::vector< boost::shared_ptr<fcl::CollisionObject> > &world)
{
    // Get the number of shapes
    int num_shapes;
    input >> num_shapes;

    std::cout << num_shapes << " shapes" << std::endl;
    for(int i = 0; i < num_shapes; ++i)
    {
        // For each shape:
        // - get the entitiy id
        std::string entity_id;
        input >> entity_id;

        std::cout << "  - " << entity_id << ":" << std::endl;

        boost::shared_ptr<fcl::CollisionObject> obj = ed_wbc::serialization::deserialize(input);
        world.push_back(obj);
    }
}


} // namespace serialization



} // namespace ed_wbc
