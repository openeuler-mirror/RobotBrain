/* Generated from orogen/lib/orogen/templates/typekit/type_info/ArrayInfo.cpp */

#include <boost/cstdint.hpp>
#include <robot_brain/typekit/BoostSerialization.hpp>
#include <rtt/types/CArrayTypeInfo.hpp>
#include <rtt/internal/carray.hpp>



namespace orogen_typekits {
    struct int8_tArrayTypeInfo :
	public RTT::types::CArrayTypeInfo< RTT::internal::carray< boost::int8_t > >
    {
        int8_tArrayTypeInfo()
            : RTT::types::CArrayTypeInfo< RTT::internal::carray< boost::int8_t > >("/int8_t[]") {}
    };

    RTT::types::TypeInfoGenerator* int8_t_ArrayTypeInfo()
    { return new int8_tArrayTypeInfo(); }
}



/* Generated from orogen/lib/orogen/templates/typekit/type_info/ArrayInfo.cpp */

#include <boost/cstdint.hpp>
#include <robot_brain/typekit/BoostSerialization.hpp>
#include <rtt/types/CArrayTypeInfo.hpp>
#include <rtt/internal/carray.hpp>



namespace orogen_typekits {
    struct uint16_tArrayTypeInfo :
	public RTT::types::CArrayTypeInfo< RTT::internal::carray< boost::uint16_t > >
    {
        uint16_tArrayTypeInfo()
            : RTT::types::CArrayTypeInfo< RTT::internal::carray< boost::uint16_t > >("/uint16_t[]") {}
    };

    RTT::types::TypeInfoGenerator* uint16_t_ArrayTypeInfo()
    { return new uint16_tArrayTypeInfo(); }
}


