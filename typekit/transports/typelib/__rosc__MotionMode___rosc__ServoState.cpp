/* Generated from orogen/lib/orogen/templates/typekit/typelib/Type.cpp */

#include <robot_brain/ethercat_frame_types.hpp>
#include <rtt/typelib/TypelibMarshaller.hpp>
#include "transports/typelib/Registration.hpp"

orogen_transports::TypelibMarshallerBase* orogen_typekits::rosc_MotionMode_TypelibMarshaller(Typelib::Registry const& registry)
{
    return new orogen_transports::TypelibMarshaller< ::rosc::MotionMode >("/rosc/MotionMode", registry);
}




/* Generated from orogen/lib/orogen/templates/typekit/typelib/Type.cpp */

#include <robot_brain/ethercat_frame_types.hpp>
#include <rtt/typelib/TypelibMarshaller.hpp>
#include "transports/typelib/Registration.hpp"

orogen_transports::TypelibMarshallerBase* orogen_typekits::rosc_ServoState_TypelibMarshaller(Typelib::Registry const& registry)
{
    return new orogen_transports::TypelibMarshaller< ::rosc::ServoState >("/rosc/ServoState", registry);
}



