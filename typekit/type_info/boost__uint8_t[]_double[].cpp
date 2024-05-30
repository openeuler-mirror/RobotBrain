/* Generated from orogen/lib/orogen/templates/typekit/type_info/ArrayInfo.cpp */

#include <boost/cstdint.hpp>
#include <robot_brain/typekit/BoostSerialization.hpp>
#include <rtt/types/CArrayTypeInfo.hpp>
#include <rtt/internal/carray.hpp>



namespace orogen_typekits {
    struct uint8_tArrayTypeInfo :
	public RTT::types::CArrayTypeInfo< RTT::internal::carray< boost::uint8_t > >
    {
        uint8_tArrayTypeInfo()
            : RTT::types::CArrayTypeInfo< RTT::internal::carray< boost::uint8_t > >("/uint8_t[]") {}
    };

    RTT::types::TypeInfoGenerator* uint8_t_ArrayTypeInfo()
    { return new uint8_tArrayTypeInfo(); }
}



/* Generated from orogen/lib/orogen/templates/typekit/type_info/ArrayInfo.cpp */


#include <robot_brain/typekit/BoostSerialization.hpp>
#include <rtt/types/CArrayTypeInfo.hpp>
#include <rtt/internal/carray.hpp>



namespace orogen_typekits {
    struct doubleArrayTypeInfo :
	public RTT::types::CArrayTypeInfo< RTT::internal::carray< double > >
    {
        doubleArrayTypeInfo()
            : RTT::types::CArrayTypeInfo< RTT::internal::carray< double > >("/double[]") {}
    };

    RTT::types::TypeInfoGenerator* double_ArrayTypeInfo()
    { return new doubleArrayTypeInfo(); }
}


