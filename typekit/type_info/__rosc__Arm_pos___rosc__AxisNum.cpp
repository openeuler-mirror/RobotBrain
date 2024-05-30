/* Generated from orogen/templates/typekit/type_info/Info.cpp */

#include <robot_brain/command_types.hpp>
#include <robot_brain/typekit/BoostSerialization.hpp>
#include <rtt/types/StructTypeInfo.hpp>



namespace orogen_typekits {
    struct rosc_Arm_posTypeInfo :
        public RTT::types::StructTypeInfo< ::rosc::Arm_pos >
    {
        rosc_Arm_posTypeInfo()
            : RTT::types::StructTypeInfo< ::rosc::Arm_pos >("/rosc/Arm_pos") {}


    };

    RTT::types::TypeInfoGenerator* rosc_Arm_pos_TypeInfo()
    { return new rosc_Arm_posTypeInfo(); }

}

/* Generated from orogen/lib/orogen/templates/typekit/TemplateInstanciation.cpp */

#include <rtt/Port.hpp>
#include <rtt/Attribute.hpp>
#include <rtt/Property.hpp>
#include <rtt/internal/DataSource.hpp>

template class RTT::base::ChannelElement< ::rosc::Arm_pos >;
template class RTT::internal::DataSource< ::rosc::Arm_pos >;
template class RTT::internal::AssignableDataSource< ::rosc::Arm_pos >;
template class RTT::internal::ValueDataSource< ::rosc::Arm_pos >;
template class RTT::OutputPort< ::rosc::Arm_pos >;
template class RTT::InputPort< ::rosc::Arm_pos >;
template class RTT::Property< ::rosc::Arm_pos >;
template class RTT::Attribute< ::rosc::Arm_pos >;




/* Generated from orogen/templates/typekit/type_info/Info.cpp */

#include <robot_brain/command_types.hpp>
#include <robot_brain/typekit/BoostSerialization.hpp>
#include <rtt/types/TemplateTypeInfo.hpp>



namespace orogen_typekits {
    struct rosc_AxisNumTypeInfo :
        public RTT::types::TemplateTypeInfo< ::rosc::AxisNum >
    {
        rosc_AxisNumTypeInfo()
            : RTT::types::TemplateTypeInfo< ::rosc::AxisNum >("/rosc/AxisNum") {}


    };

    RTT::types::TypeInfoGenerator* rosc_AxisNum_TypeInfo()
    { return new rosc_AxisNumTypeInfo(); }

}

/* Generated from orogen/lib/orogen/templates/typekit/TemplateInstanciation.cpp */

#include <rtt/Port.hpp>
#include <rtt/Attribute.hpp>
#include <rtt/Property.hpp>
#include <rtt/internal/DataSource.hpp>

template class RTT::base::ChannelElement< ::rosc::AxisNum >;
template class RTT::internal::DataSource< ::rosc::AxisNum >;
template class RTT::internal::AssignableDataSource< ::rosc::AxisNum >;
template class RTT::internal::ValueDataSource< ::rosc::AxisNum >;
template class RTT::OutputPort< ::rosc::AxisNum >;
template class RTT::InputPort< ::rosc::AxisNum >;
template class RTT::Property< ::rosc::AxisNum >;
template class RTT::Attribute< ::rosc::AxisNum >;



