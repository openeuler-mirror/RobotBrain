/* Generated from orogen/lib/orogen/templates/typekit/BoostSerialization.hpp */

#ifndef __OROGEN_GENERATED_ROBOT_BRAIN_BOOST_SERIALIZATION_HPP
#define __OROGEN_GENERATED_ROBOT_BRAIN_BOOST_SERIALIZATION_HPP

#include <robot_brain/typekit/Types.hpp>

#include <boost/serialization/serialization.hpp>
#include <boost/serialization/array.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/string.hpp>
#include <boost/serialization/map.hpp>
#include <boost/serialization/set.hpp>
#include <boost/serialization/list.hpp>
#include <boost/serialization/deque.hpp>
#include <boost/serialization/utility.hpp>


namespace boost
{
    namespace serialization
    {
        template<typename Archive>
        void serialize(Archive& a, ::rosc::Arm_pos& b, unsigned int version)
        {
            using boost::serialization::make_nvp;
            a & make_nvp("Extension", b.Extension);
a & make_nvp("Rotation", b.Rotation);
a & make_nvp("Z_axis", b.Z_axis);
        }
        template<typename Archive>
        void serialize(Archive& a, ::rosc::CommandExecResult& b, unsigned int version)
        {
            using boost::serialization::make_nvp;
            a & make_nvp("status", b.status);
a & make_nvp("tp", b.tp);
a & make_nvp("timestamp", b.timestamp);
a & make_nvp("responseParams", b.responseParams);
a & make_nvp("errcode", boost::serialization::make_array(b.errcode, 4));
        }
        template<typename Archive>
        void serialize(Archive& a, ::rosc::DigitIOFrame& b, unsigned int version)
        {
            using boost::serialization::make_nvp;
            a & make_nvp("digit_io_out", b.digit_io_out);
        }
        template<typename Archive>
        void serialize(Archive& a, ::rosc::EncoderFrame& b, unsigned int version)
        {
            using boost::serialization::make_nvp;
            a & make_nvp("target_position", boost::serialization::make_array(b.target_position, 9));
a & make_nvp("current_position", boost::serialization::make_array(b.current_position, 6));
a & make_nvp("mesg", boost::serialization::make_array(b.mesg, 100));
a & make_nvp("point_name", boost::serialization::make_array(b.point_name, 100));
a & make_nvp("time_stamp", b.time_stamp);
a & make_nvp("al_state", b.al_state);
a & make_nvp("digit_io_in", b.digit_io_in);
a & make_nvp("status", b.status);
a & make_nvp("servo_state", b.servo_state);
a & make_nvp("mode", b.mode);
        }
        template<typename Archive>
        void serialize(Archive& a, ::rosc::EthercatCtlFrame& b, unsigned int version)
        {
            using boost::serialization::make_nvp;
            a & make_nvp("operation_mode", boost::serialization::make_array(b.operation_mode, 9));
a & make_nvp("ctrl_word", boost::serialization::make_array(b.ctrl_word, 9));
a & make_nvp("target_position", boost::serialization::make_array(b.target_position, 9));
a & make_nvp("digit_io_out", b.digit_io_out);
        }
        template<typename Archive>
        void serialize(Archive& a, ::rosc::EthercatStatusFrame& b, unsigned int version)
        {
            using boost::serialization::make_nvp;
            a & make_nvp("operation_mode", boost::serialization::make_array(b.operation_mode, 9));
a & make_nvp("ctrl_word", boost::serialization::make_array(b.ctrl_word, 9));
a & make_nvp("target_position", boost::serialization::make_array(b.target_position, 9));
a & make_nvp("status_word", boost::serialization::make_array(b.status_word, 9));
a & make_nvp("current_position", boost::serialization::make_array(b.current_position, 9));
a & make_nvp("current_torque", boost::serialization::make_array(b.current_torque, 9));
a & make_nvp("last_position", boost::serialization::make_array(b.last_position, 9));
a & make_nvp("error_code", boost::serialization::make_array(b.error_code, 9));
a & make_nvp("modes_operation_display", boost::serialization::make_array(b.modes_operation_display, 9));
a & make_nvp("digit_io_out", b.digit_io_out);
a & make_nvp("digit_io_in", b.digit_io_in);
a & make_nvp("ethercat_frame_send_status", b.ethercat_frame_send_status);
a & make_nvp("al_state", b.al_state);
a & make_nvp("data_flag", b.data_flag);
        }
    }
}

#endif

