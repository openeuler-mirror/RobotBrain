/* Generated from orogen/templates/typekit/type_info/Info.cpp */

#include <robot_brain/ethercat_frame_types.hpp>
#include <robot_brain/typekit/BoostSerialization.hpp>
#include <rtt/types/StructTypeInfo.hpp>



namespace orogen_typekits {
    struct rosc_DigitIOFrameTypeInfo :
        public RTT::types::StructTypeInfo< ::rosc::DigitIOFrame >
    {
        rosc_DigitIOFrameTypeInfo()
            : RTT::types::StructTypeInfo< ::rosc::DigitIOFrame >("/rosc/DigitIOFrame") {}


    };

    RTT::types::TypeInfoGenerator* rosc_DigitIOFrame_TypeInfo()
    { return new rosc_DigitIOFrameTypeInfo(); }

}

/* Generated from orogen/lib/orogen/templates/typekit/TemplateInstanciation.cpp */

#include <rtt/Port.hpp>
#include <rtt/Attribute.hpp>
#include <rtt/Property.hpp>
#include <rtt/internal/DataSource.hpp>

template class RTT::base::ChannelElement< ::rosc::DigitIOFrame >;
template class RTT::internal::DataSource< ::rosc::DigitIOFrame >;
template class RTT::internal::AssignableDataSource< ::rosc::DigitIOFrame >;
template class RTT::internal::ValueDataSource< ::rosc::DigitIOFrame >;
template class RTT::OutputPort< ::rosc::DigitIOFrame >;
template class RTT::InputPort< ::rosc::DigitIOFrame >;
template class RTT::Property< ::rosc::DigitIOFrame >;
template class RTT::Attribute< ::rosc::DigitIOFrame >;




/* Generated from orogen/templates/typekit/type_info/Info.cpp */

#include <robot_brain/ethercat_frame_types.hpp>
#include <robot_brain/typekit/BoostSerialization.hpp>
#include <rtt/types/StructTypeInfo.hpp>



namespace orogen_typekits {
    struct rosc_EncoderFrameTypeInfo :
        public RTT::types::StructTypeInfo< ::rosc::EncoderFrame >
    {
        rosc_EncoderFrameTypeInfo()
            : RTT::types::StructTypeInfo< ::rosc::EncoderFrame >("/rosc/EncoderFrame") {}


    };

    RTT::types::TypeInfoGenerator* rosc_EncoderFrame_TypeInfo()
    { return new rosc_EncoderFrameTypeInfo(); }

}

/* Generated from orogen/lib/orogen/templates/typekit/TemplateInstanciation.cpp */

#include <rtt/Port.hpp>
#include <rtt/Attribute.hpp>
#include <rtt/Property.hpp>
#include <rtt/internal/DataSource.hpp>

template class RTT::base::ChannelElement< ::rosc::EncoderFrame >;
template class RTT::internal::DataSource< ::rosc::EncoderFrame >;
template class RTT::internal::AssignableDataSource< ::rosc::EncoderFrame >;
template class RTT::internal::ValueDataSource< ::rosc::EncoderFrame >;
template class RTT::OutputPort< ::rosc::EncoderFrame >;
template class RTT::InputPort< ::rosc::EncoderFrame >;
template class RTT::Property< ::rosc::EncoderFrame >;
template class RTT::Attribute< ::rosc::EncoderFrame >;



