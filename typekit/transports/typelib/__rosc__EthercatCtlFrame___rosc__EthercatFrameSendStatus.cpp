/* Generated from orogen/lib/orogen/templates/typekit/typelib/Type.cpp */

#include <robot_brain/ethercat_frame_types.hpp>
#include <rtt/typelib/TypelibMarshaller.hpp>
#include "transports/typelib/Registration.hpp"

orogen_transports::TypelibMarshallerBase* orogen_typekits::rosc_EthercatCtlFrame_TypelibMarshaller(Typelib::Registry const& registry)
{
    return new orogen_transports::TypelibMarshaller< ::rosc::EthercatCtlFrame >("/rosc/EthercatCtlFrame", registry);
}




/* Generated from orogen/lib/orogen/templates/typekit/typelib/Type.cpp */

#include <robot_brain/ethercat_frame_types.hpp>
#include <rtt/typelib/TypelibMarshaller.hpp>
#include "transports/typelib/Registration.hpp"

orogen_transports::TypelibMarshallerBase* orogen_typekits::rosc_EthercatFrameSendStatus_TypelibMarshaller(Typelib::Registry const& registry)
{
    return new orogen_transports::TypelibMarshaller< ::rosc::EthercatFrameSendStatus >("/rosc/EthercatFrameSendStatus", registry);
}



