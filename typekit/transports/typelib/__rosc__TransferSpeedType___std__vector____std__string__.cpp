/* Generated from orogen/lib/orogen/templates/typekit/typelib/Type.cpp */

#include <robot_brain/command_types.hpp>
#include <rtt/typelib/TypelibMarshaller.hpp>
#include "transports/typelib/Registration.hpp"

orogen_transports::TypelibMarshallerBase* orogen_typekits::rosc_TransferSpeedType_TypelibMarshaller(Typelib::Registry const& registry)
{
    return new orogen_transports::TypelibMarshaller< ::rosc::TransferSpeedType >("/rosc/TransferSpeedType", registry);
}




/* Generated from orogen/lib/orogen/templates/typekit/typelib/Type.cpp */

#include <boost/cstdint.hpp>
#include <string>
#include <vector>
#include <rtt/typelib/TypelibMarshaller.hpp>
#include "transports/typelib/Registration.hpp"

orogen_transports::TypelibMarshallerBase* orogen_typekits::std_vector_LT__std_string_GT__TypelibMarshaller(Typelib::Registry const& registry)
{
    return new orogen_transports::TypelibMarshaller< ::std::vector< ::std::string > >("/std/vector</std/string>", registry);
}



