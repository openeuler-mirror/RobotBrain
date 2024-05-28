/* Generated from orogen/lib/orogen/templates/typekit/typelib/Type.cpp */

#include <robot_brain/command_types.hpp>
#include <rtt/typelib/TypelibMarshaller.hpp>
#include "transports/typelib/Registration.hpp"

orogen_transports::TypelibMarshallerBase* orogen_typekits::rosc_CommandExecResult_TypelibMarshaller(Typelib::Registry const& registry)
{
    return new orogen_transports::TypelibMarshaller< ::rosc::CommandExecResult >("/rosc/CommandExecResult", registry);
}




/* Generated from orogen/lib/orogen/templates/typekit/typelib/Type.cpp */

#include <robot_brain/command_types.hpp>
#include <rtt/typelib/TypelibMarshaller.hpp>
#include "transports/typelib/Registration.hpp"

orogen_transports::TypelibMarshallerBase* orogen_typekits::rosc_CommandType_TypelibMarshaller(Typelib::Registry const& registry)
{
    return new orogen_transports::TypelibMarshaller< ::rosc::CommandType >("/rosc/CommandType", registry);
}



