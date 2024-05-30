/* Generated from orogen/templates/typekit/type_info/Info.cpp */

#include <robot_brain/command_types.hpp>
#include <robot_brain/typekit/BoostSerialization.hpp>
#include <rtt/types/TemplateTypeInfo.hpp>



namespace orogen_typekits {
    struct rosc_TransferSpeedTypeTypeInfo :
        public RTT::types::TemplateTypeInfo< ::rosc::TransferSpeedType >
    {
        rosc_TransferSpeedTypeTypeInfo()
            : RTT::types::TemplateTypeInfo< ::rosc::TransferSpeedType >("/rosc/TransferSpeedType") {}


    };

    RTT::types::TypeInfoGenerator* rosc_TransferSpeedType_TypeInfo()
    { return new rosc_TransferSpeedTypeTypeInfo(); }

}

/* Generated from orogen/lib/orogen/templates/typekit/TemplateInstanciation.cpp */

#include <rtt/Port.hpp>
#include <rtt/Attribute.hpp>
#include <rtt/Property.hpp>
#include <rtt/internal/DataSource.hpp>

template class RTT::base::ChannelElement< ::rosc::TransferSpeedType >;
template class RTT::internal::DataSource< ::rosc::TransferSpeedType >;
template class RTT::internal::AssignableDataSource< ::rosc::TransferSpeedType >;
template class RTT::internal::ValueDataSource< ::rosc::TransferSpeedType >;
template class RTT::OutputPort< ::rosc::TransferSpeedType >;
template class RTT::InputPort< ::rosc::TransferSpeedType >;
template class RTT::Property< ::rosc::TransferSpeedType >;
template class RTT::Attribute< ::rosc::TransferSpeedType >;




/* Generated from orogen/templates/typekit/type_info/Info.cpp */

#include <boost/cstdint.hpp>
#include <string>
#include <vector>
#include <robot_brain/typekit/BoostSerialization.hpp>
#include <rtt/types/SequenceTypeInfo.hpp>



namespace orogen_typekits {
    struct std_vector_LT__std_string_GT_TypeInfo :
        public RTT::types::SequenceTypeInfo< ::std::vector< ::std::string > >
    {
        std_vector_LT__std_string_GT_TypeInfo()
            : RTT::types::SequenceTypeInfo< ::std::vector< ::std::string > >("/std/vector</std/string>") {}


    };

    RTT::types::TypeInfoGenerator* std_vector_LT__std_string_GT__TypeInfo()
    { return new std_vector_LT__std_string_GT_TypeInfo(); }

}

/* Generated from orogen/lib/orogen/templates/typekit/TemplateInstanciation.cpp */

#include <rtt/Port.hpp>
#include <rtt/Attribute.hpp>
#include <rtt/Property.hpp>
#include <rtt/internal/DataSource.hpp>

template class RTT::base::ChannelElement< ::std::vector< ::std::string > >;
template class RTT::internal::DataSource< ::std::vector< ::std::string > >;
template class RTT::internal::AssignableDataSource< ::std::vector< ::std::string > >;
template class RTT::internal::ValueDataSource< ::std::vector< ::std::string > >;
template class RTT::OutputPort< ::std::vector< ::std::string > >;
template class RTT::InputPort< ::std::vector< ::std::string > >;
template class RTT::Property< ::std::vector< ::std::string > >;
template class RTT::Attribute< ::std::vector< ::std::string > >;



