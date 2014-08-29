#include "probe.h"

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

    int sum = i1 + i2;

    std::cout << "ExampleProbe received: " << i1 << " " << i2 << std::endl;

    res << sum << "Hello world!";
}

ED_REGISTER_PLUGIN(ExampleProbe)
