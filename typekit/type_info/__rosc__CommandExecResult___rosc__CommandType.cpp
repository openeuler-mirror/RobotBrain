/* Generated from orogen/templates/typekit/type_info/Info.cpp */

#include <robot_brain/command_types.hpp>
#include <robot_brain/typekit/BoostSerialization.hpp>
#include <rtt/types/StructTypeInfo.hpp>



namespace orogen_typekits {
    struct rosc_CommandExecResultTypeInfo :
        public RTT::types::StructTypeInfo< ::rosc::CommandExecResult >
    {
        rosc_CommandExecResultTypeInfo()
            : RTT::types::StructTypeInfo< ::rosc::CommandExecResult >("/rosc/CommandExecResult") {}


    };

    RTT::types::TypeInfoGenerator* rosc_CommandExecResult_TypeInfo()
    { return new rosc_CommandExecResultTypeInfo(); }

}

/* Generated from orogen/lib/orogen/templates/typekit/TemplateInstanciation.cpp */

#include <rtt/Port.hpp>
#include <rtt/Attribute.hpp>
#include <rtt/Property.hpp>
#include <rtt/internal/DataSource.hpp>

template class RTT::base::ChannelElement< ::rosc::CommandExecResult >;
template class RTT::internal::DataSource< ::rosc::CommandExecResult >;
template class RTT::internal::AssignableDataSource< ::rosc::CommandExecResult >;
template class RTT::internal::ValueDataSource< ::rosc::CommandExecResult >;
template class RTT::OutputPort< ::rosc::CommandExecResult >;
template class RTT::InputPort< ::rosc::CommandExecResult >;
template class RTT::Property< ::rosc::CommandExecResult >;
template class RTT::Attribute< ::rosc::CommandExecResult >;




/* Generated from orogen/templates/typekit/type_info/Info.cpp */

#include <robot_brain/command_types.hpp>
#include <robot_brain/typekit/BoostSerialization.hpp>
#include <rtt/types/TemplateTypeInfo.hpp>



namespace orogen_typekits {
    struct rosc_CommandTypeTypeInfo :
        public RTT::types::TemplateTypeInfo< ::rosc::CommandType >
    {
        rosc_CommandTypeTypeInfo()
            : RTT::types::TemplateTypeInfo< ::rosc::CommandType >("/rosc/CommandType") {}


    };

    RTT::types::TypeInfoGenerator* rosc_CommandType_TypeInfo()
    { return new rosc_CommandTypeTypeInfo(); }

}

/* Generated from orogen/lib/orogen/templates/typekit/TemplateInstanciation.cpp */

#include <rtt/Port.hpp>
#include <rtt/Attribute.hpp>
#include <rtt/Property.hpp>
#include <rtt/internal/DataSource.hpp>

template class RTT::base::ChannelElement< ::rosc::CommandType >;
template class RTT::internal::DataSource< ::rosc::CommandType >;
template class RTT::internal::AssignableDataSource< ::rosc::CommandType >;
template class RTT::internal::ValueDataSource< ::rosc::CommandType >;
template class RTT::OutputPort< ::rosc::CommandType >;
template class RTT::InputPort< ::rosc::CommandType >;
template class RTT::Property< ::rosc::CommandType >;
template class RTT::Attribute< ::rosc::CommandType >;



