#ifndef SERIALIZATION_H
#define SERIALIZATION_H

#include <boost/shared_ptr.hpp>
#include <tue/serialization/archive.h>
#include <tue/serialization/input_archive.h>
#include <tue/serialization/output_archive.h>

#include <geolib/datatypes.h>
#include <ed/world_model.h>

#include <fcl/collision_object.h>

namespace ed_wbc {


namespace serialization {

bool serialize(const geo::Shape& shape, tue::serialization::OutputArchive &output);

bool serialize(const ed::WorldModel& world, tue::serialization::OutputArchive& output);

const boost::shared_ptr<fcl::CollisionObject> deserialize(tue::serialization::Archive& input);

}


} // namespace

#endif // SERIALIZATION_H
