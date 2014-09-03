#include "wbc_probe.h"
#include "serialization.h"

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

    serialization::serializeCollisionWorld(world, res);
}

// Make sure ED can find this probe within the library
ED_REGISTER_PLUGIN(ShapeProbe)
