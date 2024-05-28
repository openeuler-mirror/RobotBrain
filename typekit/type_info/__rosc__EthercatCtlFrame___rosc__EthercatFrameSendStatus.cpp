/* Generated from orogen/templates/typekit/type_info/Info.cpp */

#include <robot_brain/ethercat_frame_types.hpp>
#include <robot_brain/typekit/BoostSerialization.hpp>
#include <rtt/types/StructTypeInfo.hpp>



namespace orogen_typekits {
    struct rosc_EthercatCtlFrameTypeInfo :
        public RTT::types::StructTypeInfo< ::rosc::EthercatCtlFrame >
    {
        rosc_EthercatCtlFrameTypeInfo()
            : RTT::types::StructTypeInfo< ::rosc::EthercatCtlFrame >("/rosc/EthercatCtlFrame") {}


    };

    RTT::types::TypeInfoGenerator* rosc_EthercatCtlFrame_TypeInfo()
    { return new rosc_EthercatCtlFrameTypeInfo(); }

}

/* Generated from orogen/lib/orogen/templates/typekit/TemplateInstanciation.cpp */

#include <rtt/Port.hpp>
#include <rtt/Attribute.hpp>
#include <rtt/Property.hpp>
#include <rtt/internal/DataSource.hpp>

template class RTT::base::ChannelElement< ::rosc::EthercatCtlFrame >;
template class RTT::internal::DataSource< ::rosc::EthercatCtlFrame >;
template class RTT::internal::AssignableDataSource< ::rosc::EthercatCtlFrame >;
template class RTT::internal::ValueDataSource< ::rosc::EthercatCtlFrame >;
template class RTT::OutputPort< ::rosc::EthercatCtlFrame >;
template class RTT::InputPort< ::rosc::EthercatCtlFrame >;
template class RTT::Property< ::rosc::EthercatCtlFrame >;
template class RTT::Attribute< ::rosc::EthercatCtlFrame >;




/* Generated from orogen/templates/typekit/type_info/Info.cpp */

#include <robot_brain/ethercat_frame_types.hpp>
#include <robot_brain/typekit/BoostSerialization.hpp>
#include <rtt/types/TemplateTypeInfo.hpp>



namespace orogen_typekits {
    struct rosc_EthercatFrameSendStatusTypeInfo :
        public RTT::types::TemplateTypeInfo< ::rosc::EthercatFrameSendStatus >
    {
        rosc_EthercatFrameSendStatusTypeInfo()
            : RTT::types::TemplateTypeInfo< ::rosc::EthercatFrameSendStatus >("/rosc/EthercatFrameSendStatus") {}


    };

    RTT::types::TypeInfoGenerator* rosc_EthercatFrameSendStatus_TypeInfo()
    { return new rosc_EthercatFrameSendStatusTypeInfo(); }

}

/* Generated from orogen/lib/orogen/templates/typekit/TemplateInstanciation.cpp */

#include <rtt/Port.hpp>
#include <rtt/Attribute.hpp>
#include <rtt/Property.hpp>
#include <rtt/internal/DataSource.hpp>

template class RTT::base::ChannelElement< ::rosc::EthercatFrameSendStatus >;
template class RTT::internal::DataSource< ::rosc::EthercatFrameSendStatus >;
template class RTT::internal::AssignableDataSource< ::rosc::EthercatFrameSendStatus >;
template class RTT::internal::ValueDataSource< ::rosc::EthercatFrameSendStatus >;
template class RTT::OutputPort< ::rosc::EthercatFrameSendStatus >;
template class RTT::InputPort< ::rosc::EthercatFrameSendStatus >;
template class RTT::Property< ::rosc::EthercatFrameSendStatus >;
template class RTT::Attribute< ::rosc::EthercatFrameSendStatus >;



