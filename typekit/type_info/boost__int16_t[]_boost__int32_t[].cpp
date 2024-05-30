/* Generated from orogen/lib/orogen/templates/typekit/type_info/ArrayInfo.cpp */

#include <boost/cstdint.hpp>
#include <robot_brain/typekit/BoostSerialization.hpp>
#include <rtt/types/CArrayTypeInfo.hpp>
#include <rtt/internal/carray.hpp>



namespace orogen_typekits {
    struct int16_tArrayTypeInfo :
	public RTT::types::CArrayTypeInfo< RTT::internal::carray< boost::int16_t > >
    {
        int16_tArrayTypeInfo()
            : RTT::types::CArrayTypeInfo< RTT::internal::carray< boost::int16_t > >("/int16_t[]") {}
    };

    RTT::types::TypeInfoGenerator* int16_t_ArrayTypeInfo()
    { return new int16_tArrayTypeInfo(); }
}



/* Generated from orogen/lib/orogen/templates/typekit/type_info/ArrayInfo.cpp */

#include <boost/cstdint.hpp>
#include <robot_brain/typekit/BoostSerialization.hpp>
#include <rtt/types/CArrayTypeInfo.hpp>
#include <rtt/internal/carray.hpp>



namespace orogen_typekits {
    struct int32_tArrayTypeInfo :
	public RTT::types::CArrayTypeInfo< RTT::internal::carray< boost::int32_t > >
    {
        int32_tArrayTypeInfo()
            : RTT::types::CArrayTypeInfo< RTT::internal::carray< boost::int32_t > >("/int32_t[]") {}
    };

    RTT::types::TypeInfoGenerator* int32_t_ArrayTypeInfo()
    { return new int32_tArrayTypeInfo(); }
}


