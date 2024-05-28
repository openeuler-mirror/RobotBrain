/* Generated from orogen/templates/typekit/type_info/Info.cpp */

#include <robot_brain/ethercat_frame_types.hpp>
#include <robot_brain/typekit/BoostSerialization.hpp>
#include <rtt/types/TemplateTypeInfo.hpp>



namespace orogen_typekits {
    struct rosc_MotionModeTypeInfo :
        public RTT::types::TemplateTypeInfo< ::rosc::MotionMode >
    {
        rosc_MotionModeTypeInfo()
            : RTT::types::TemplateTypeInfo< ::rosc::MotionMode >("/rosc/MotionMode") {}


    };

    RTT::types::TypeInfoGenerator* rosc_MotionMode_TypeInfo()
    { return new rosc_MotionModeTypeInfo(); }

}

/* Generated from orogen/lib/orogen/templates/typekit/TemplateInstanciation.cpp */

#include <rtt/Port.hpp>
#include <rtt/Attribute.hpp>
#include <rtt/Property.hpp>
#include <rtt/internal/DataSource.hpp>

template class RTT::base::ChannelElement< ::rosc::MotionMode >;
template class RTT::internal::DataSource< ::rosc::MotionMode >;
template class RTT::internal::AssignableDataSource< ::rosc::MotionMode >;
template class RTT::internal::ValueDataSource< ::rosc::MotionMode >;
template class RTT::OutputPort< ::rosc::MotionMode >;
template class RTT::InputPort< ::rosc::MotionMode >;
template class RTT::Property< ::rosc::MotionMode >;
template class RTT::Attribute< ::rosc::MotionMode >;




/* Generated from orogen/templates/typekit/type_info/Info.cpp */

#include <robot_brain/ethercat_frame_types.hpp>
#include <robot_brain/typekit/BoostSerialization.hpp>
#include <rtt/types/TemplateTypeInfo.hpp>



namespace orogen_typekits {
    struct rosc_ServoStateTypeInfo :
        public RTT::types::TemplateTypeInfo< ::rosc::ServoState >
    {
        rosc_ServoStateTypeInfo()
            : RTT::types::TemplateTypeInfo< ::rosc::ServoState >("/rosc/ServoState") {}


    };

    RTT::types::TypeInfoGenerator* rosc_ServoState_TypeInfo()
    { return new rosc_ServoStateTypeInfo(); }

}

/* Generated from orogen/lib/orogen/templates/typekit/TemplateInstanciation.cpp */

#include <rtt/Port.hpp>
#include <rtt/Attribute.hpp>
#include <rtt/Property.hpp>
#include <rtt/internal/DataSource.hpp>

template class RTT::base::ChannelElement< ::rosc::ServoState >;
template class RTT::internal::DataSource< ::rosc::ServoState >;
template class RTT::internal::AssignableDataSource< ::rosc::ServoState >;
template class RTT::internal::ValueDataSource< ::rosc::ServoState >;
template class RTT::OutputPort< ::rosc::ServoState >;
template class RTT::InputPort< ::rosc::ServoState >;
template class RTT::Property< ::rosc::ServoState >;
template class RTT::Attribute< ::rosc::ServoState >;



