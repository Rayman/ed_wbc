#ifndef SERIALIZATION_H
#define SERIALIZATION_H

#include <geolib/datatypes.h>
#include <fcl/collision_object.h>

#include <boost/shared_ptr.hpp>

namespace ed_wbc {


namespace serialization {

bool serialize(const geo::Shape& shape, std::ostream& output);

const boost::shared_ptr<fcl::CollisionObject> deserialize(std::istream& input);

}


} // namespace

#endif // SERIALIZATION_H
