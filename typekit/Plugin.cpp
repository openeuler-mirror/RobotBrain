/* Generated from orogen/lib/orogen/templates/typekit/Plugin.cpp */

// First load all RTT interfaces so that we get all "extern template"
// declarations in the TypekitImpl files
#include <robot_brain/typekit/Plugin.hpp>

#include <boost/lexical_cast.hpp>

#include <rtt/types/TypeInfoRepository.hpp>
#include "type_info/Registration.hpp"

using namespace RTT;

orogen_typekits::robot_brainTypekitPlugin::robot_brainTypekitPlugin()
{}

orogen_typekits::robot_brainTypekitPlugin::~robot_brainTypekitPlugin()
{}


#define TYPEKIT_PACKAGE_NAME_aux0(target) #target
#define TYPEKIT_PACKAGE_NAME_aux(target) "robot_brain-typekit-" TYPEKIT_PACKAGE_NAME_aux0(target)
#define TYPEKIT_PACKAGE_NAME TYPEKIT_PACKAGE_NAME_aux(OROCOS_TARGET)
bool orogen_typekits::robot_brainTypekitPlugin::loadTypes()
{
    RTT::types::TypeInfoRepository::shared_ptr ti_repository = RTT::types::TypeInfoRepository::Instance();

    
    RTT::types::TypeInfoGenerator* ti = 0;
    
        
    ti = double_ArrayTypeInfo();
    ti_repository->addType( ti );
        
    
        
    ti = double_ArrayTypeInfo();
    ti_repository->addType( ti );
        
    
        
    ti = int16_t_ArrayTypeInfo();
    ti_repository->addType( ti );
        
    
        
    ti = int32_t_ArrayTypeInfo();
    ti_repository->addType( ti );
        
    
        
    ti = int8_t_ArrayTypeInfo();
    ti_repository->addType( ti );
        
    
        
    ti = int8_t_ArrayTypeInfo();
    ti_repository->addType( ti );
        
    
        
    ti = rosc_Arm_pos_TypeInfo();
    ti_repository->addType( ti );
        
    
        
    ti = rosc_AxisNum_TypeInfo();
    ti_repository->addType( ti );
        
    
        
    ti = rosc_CommandExecResult_TypeInfo();
    ti_repository->addType( ti );
        
    
        
    ti = rosc_CommandType_TypeInfo();
    ti_repository->addType( ti );
        
    
        
    ti = rosc_DigitIOFrame_TypeInfo();
    ti_repository->addType( ti );
        
    
        
    ti = rosc_EncoderFrame_TypeInfo();
    ti_repository->addType( ti );
        
    
        
    ti = rosc_EthercatCtlFrame_TypeInfo();
    ti_repository->addType( ti );
        
    
        
    ti = rosc_EthercatFrameSendStatus_TypeInfo();
    ti_repository->addType( ti );
        
    
        
    ti = rosc_EthercatStatusFrame_TypeInfo();
    ti_repository->addType( ti );
        
    
        
    ti = rosc_ExecStatus_TypeInfo();
    ti_repository->addType( ti );
        
    
        
    ti = rosc_MotionMode_TypeInfo();
    ti_repository->addType( ti );
        
    
        
    ti = rosc_ServoState_TypeInfo();
    ti_repository->addType( ti );
        
    
        
    ti = rosc_TransferSpeedType_TypeInfo();
    ti_repository->addType( ti );
        
    
        
    ti = std_vector_LT__std_string_GT__TypeInfo();
    ti_repository->addType( ti );
        
    
        
    ti = uint16_t_ArrayTypeInfo();
    ti_repository->addType( ti );
        
    
        
    ti = uint8_t_ArrayTypeInfo();
    ti_repository->addType( ti );
        
    
    

    return true;
}

bool orogen_typekits::robot_brainTypekitPlugin::loadOperators()
{ return true; }
bool orogen_typekits::robot_brainTypekitPlugin::loadConstructors()
{ return true; }
std::string orogen_typekits::robot_brainTypekitPlugin::getName()
{ return "robot_brain"; }

ORO_TYPEKIT_PLUGIN(orogen_typekits::robot_brainTypekitPlugin);

