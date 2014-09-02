#ifndef ED_EXAMPLES_SHAPE_PROBE_H_
#define ED_EXAMPLES_SHAPE_PROBE_H_

#include <ed/io/transport/probe.h>

class ShapeProbe : public ed::Probe
{

public:

    ShapeProbe();

    virtual ~ShapeProbe();

    void configure(tue::Configuration config);

    void process(const ed::WorldModel& world,
                 ed::UpdateRequest& update,
                 tue::serialization::InputArchive& req,
                 tue::serialization::OutputArchive& res);

};

#endif
