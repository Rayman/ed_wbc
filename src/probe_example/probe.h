#ifndef ED_EXAMPLES_PROBE_EXAMPLE_PROBE_H_
#define ED_EXAMPLES_PROBE_EXAMPLE_PROBE_H_

#include <ed/io/transport/probe.h>

class ExampleProbe : public ed::Probe
{

public:

    ExampleProbe();

    virtual ~ExampleProbe();

    void configure(tue::Configuration config);

    void process(const ed::WorldModel& world,
                 ed::UpdateRequest& update,
                 tue::serialization::InputArchive& req,
                 tue::serialization::OutputArchive& res);

};

#endif
