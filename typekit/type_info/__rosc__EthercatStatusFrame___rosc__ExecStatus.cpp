/* Generated from orogen/templates/typekit/type_info/Info.cpp */

#include <robot_brain/ethercat_frame_types.hpp>
#include <robot_brain/typekit/BoostSerialization.hpp>
#include <rtt/types/StructTypeInfo.hpp>



namespace orogen_typekits {
    struct rosc_EthercatStatusFrameTypeInfo :
        public RTT::types::StructTypeInfo< ::rosc::EthercatStatusFrame >
    {
        rosc_EthercatStatusFrameTypeInfo()
            : RTT::types::StructTypeInfo< ::rosc::EthercatStatusFrame >("/rosc/EthercatStatusFrame") {}


    };

    RTT::types::TypeInfoGenerator* rosc_EthercatStatusFrame_TypeInfo()
    { return new rosc_EthercatStatusFrameTypeInfo(); }

}

/* Generated from orogen/lib/orogen/templates/typekit/TemplateInstanciation.cpp */

#include <rtt/Port.hpp>
#include <rtt/Attribute.hpp>
#include <rtt/Property.hpp>
#include <rtt/internal/DataSource.hpp>

template class RTT::base::ChannelElement< ::rosc::EthercatStatusFrame >;
template class RTT::internal::DataSource< ::rosc::EthercatStatusFrame >;
template class RTT::internal::AssignableDataSource< ::rosc::EthercatStatusFrame >;
template class RTT::internal::ValueDataSource< ::rosc::EthercatStatusFrame >;
template class RTT::OutputPort< ::rosc::EthercatStatusFrame >;
template class RTT::InputPort< ::rosc::EthercatStatusFrame >;
template class RTT::Property< ::rosc::EthercatStatusFrame >;
template class RTT::Attribute< ::rosc::EthercatStatusFrame >;




/* Generated from orogen/templates/typekit/type_info/Info.cpp */

#include <robot_brain/command_types.hpp>
#include <robot_brain/typekit/BoostSerialization.hpp>
#include <rtt/types/TemplateTypeInfo.hpp>



namespace orogen_typekits {
    struct rosc_ExecStatusTypeInfo :
        public RTT::types::TemplateTypeInfo< ::rosc::ExecStatus >
    {
        rosc_ExecStatusTypeInfo()
            : RTT::types::TemplateTypeInfo< ::rosc::ExecStatus >("/rosc/ExecStatus") {}


    };

    RTT::types::TypeInfoGenerator* rosc_ExecStatus_TypeInfo()
    { return new rosc_ExecStatusTypeInfo(); }

}

/* Generated from orogen/lib/orogen/templates/typekit/TemplateInstanciation.cpp */

#include <rtt/Port.hpp>
#include <rtt/Attribute.hpp>
#include <rtt/Property.hpp>
#include <rtt/internal/DataSource.hpp>

template class RTT::base::ChannelElement< ::rosc::ExecStatus >;
template class RTT::internal::DataSource< ::rosc::ExecStatus >;
template class RTT::internal::AssignableDataSource< ::rosc::ExecStatus >;
template class RTT::internal::ValueDataSource< ::rosc::ExecStatus >;
template class RTT::OutputPort< ::rosc::ExecStatus >;
template class RTT::InputPort< ::rosc::ExecStatus >;
template class RTT::Property< ::rosc::ExecStatus >;
template class RTT::Attribute< ::rosc::ExecStatus >;



