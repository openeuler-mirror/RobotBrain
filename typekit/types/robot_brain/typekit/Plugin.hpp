/* Generated from orogen/lib/orogen/templates/typekit/Plugin.hpp */

#ifndef __OROGEN_GENERATED_ROBOT_BRAIN_TYPEKIT_HPP
#define __OROGEN_GENERATED_ROBOT_BRAIN_TYPEKIT_HPP

#include <rtt/types/TypekitPlugin.hpp>

namespace Typelib {
    class Registry;
}

namespace orogen_typekits {
    class robot_brainTypekitPlugin
        : public RTT::types::TypekitPlugin
    {
        Typelib::Registry* m_registry;

    public:
        robot_brainTypekitPlugin();
        ~robot_brainTypekitPlugin();
        bool loadTypes();
        bool loadOperators();
        bool loadConstructors();
        std::string getName();
    };

    extern robot_brainTypekitPlugin robot_brainTypekit;
}

#endif


