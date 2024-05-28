/* Generated from orogen/lib/orogen/templates/typekit/typelib/Type.cpp */

#include <robot_brain/ethercat_frame_types.hpp>
#include <rtt/typelib/TypelibMarshaller.hpp>
#include "transports/typelib/Registration.hpp"

orogen_transports::TypelibMarshallerBase* orogen_typekits::rosc_EthercatStatusFrame_TypelibMarshaller(Typelib::Registry const& registry)
{
    return new orogen_transports::TypelibMarshaller< ::rosc::EthercatStatusFrame >("/rosc/EthercatStatusFrame", registry);
}




/* Generated from orogen/lib/orogen/templates/typekit/typelib/Type.cpp */

#include <robot_brain/command_types.hpp>
#include <rtt/typelib/TypelibMarshaller.hpp>
#include "transports/typelib/Registration.hpp"

orogen_transports::TypelibMarshallerBase* orogen_typekits::rosc_ExecStatus_TypelibMarshaller(Typelib::Registry const& registry)
{
    return new orogen_transports::TypelibMarshaller< ::rosc::ExecStatus >("/rosc/ExecStatus", registry);
}



