#include "serialization.h"

#include <geolib/Shape.h>
#include <ed/entity.h>
#include <fcl/BVH/BVH_model.h>

#include <ed/world_model/transform_crawler.h>

#include <ros/console.h>

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

    // Loop over all world entities and find entities that have a shape with their pose relative to MAP
    std::vector<ed::EntityConstPtr> shape_entities;
    std::vector<geo::Pose3D> entity_poses;
    for(ed::world_model::TransformCrawler tc(world, "map", world.latestTime()); tc.hasNext(); tc.next())
    {
        if (tc.entity()->shape())
        {
            shape_entities.push_back(tc.entity());
            entity_poses.push_back(tc.transform());
        }
    }

    // Add the number of entities to the response
    output << (int)shape_entities.size();
    for(unsigned int i = 0; i < shape_entities.size(); ++i)
    {
        const ed::EntityConstPtr& e = shape_entities[i];
        geo::ShapeConstPtr shape = e->shape();

        // Add the id of the entity to the response
        output << e->id().str();

        // serialize the pose
        const geo::Pose3D& pose = entity_poses[i];
        output << (float) pose.t.x;
        output << (float) pose.t.y;
        output << (float) pose.t.z;
        for (int i = 0; i < 9; i++) {
            output << (float) pose.R.m[i];
        }

        serialization::serialize(*shape, output);
    }
}

boost::shared_ptr<fcl::CollisionGeometry> deserialize(tue::serialization::Archive &input)
{
    std::vector<fcl::Vec3f>    vertices;
    std::vector<fcl::Triangle> triangles;

    // get the number of vertices, and content of these vertices
    int num_vertices;
    input >> num_vertices;
    vertices.reserve(num_vertices);
    for(int j = 0; j < num_vertices; ++j)
    {
        float x, y, z;
        input >> x >> y >> z;
        vertices.push_back(fcl::Vec3f(x, y, z));
    }

    // get the number of triangles, and content of these triangles
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

    return boost::shared_ptr<fcl::CollisionGeometry>(model);
}

void deserializeCollisionWorld(tue::serialization::Archive &input, std::vector<WorldCollisionObject> &world)
{
    int num_shapes;
    input >> num_shapes;

    for(int i = 0; i < num_shapes; ++i)
    {
        std::string entity_id;
        input >> entity_id;

        // deserialize the pose

        float x, y, z;
        input >> x >> y >> z;
        fcl::Vec3f T(x, y, z);

        float xx, xy, xz, yx, yy, yz, zx, zy, zz;
        input >> xx >> xy >> xz >> yx >> yy >> yz >> zx >> zy >> zz;
        fcl::Matrix3f R(xx, xy, xz, yx, yy, yz, zx, zy, zz);

        boost::shared_ptr<fcl::CollisionGeometry> geom = ed_wbc::serialization::deserialize(input);
        boost::shared_ptr<fcl::CollisionObject> obj(new fcl::CollisionObject(geom, R, T));

        world.push_back(obj);
    }
}


} // namespace serialization



} // namespace ed_wbc
