%module "xarm"
%{
#include "../lib/xArm-CPLUS-SDK/include/xarm/wrapper/xarm_api_simple.h"
%}
// needed to support std::string types
%include "std_string.i"
// 1st solution for overloaded functions: just ignore
%ignore set_position(fp32 pose[6], fp32 radius = -1, bool wait = false, fp32 timeout = NO_TIMEOUT);
%ignore set_position(fp32 pose[6], bool wait = false, fp32 timeout = NO_TIMEOUT);
%ignore set_tool_position(fp32 pose[6], bool wait = false, fp32 timeout = NO_TIMEOUT);
%ignore set_servo_angle(fp32 angles[7], bool wait = false, fp32 timeout = NO_TIMEOUT, fp32 radius = -1);
%ignore set_servo_angle(int servo_id, fp32 angle, fp32 speed = 0, fp32 acc = 0, fp32 mvtime = 0, bool wait = false, fp32 timeout = NO_TIMEOUT, fp32 radius = -1);
%ignore set_servo_angle(int servo_id, fp32 angle, bool wait = false, fp32 timeout = NO_TIMEOUT, fp32 radius = -1);
%ignore move_gohome(bool wait = false, fp32 timeout = NO_TIMEOUT);
/*
%ignore register_report_data_callback(std::function<void (XArmReportData *)> callback);
%ignore register_report_location_callback(std::function<void (const fp32*, const fp32*)> callback);
%ignore register_connect_changed_callback(std::function<void (bool, bool)> callback);
%ignore register_state_changed_callback(std::function<void (int)> callback);
%ignore register_mode_changed_callback(std::function<void (int)> callback);
%ignore register_mtable_mtbrake_changed_callback(std::function<void (int, int)> callback);
%ignore register_error_warn_changed_callback(std::function<void (int, int)> callback);
%ignore register_cmdnum_changed_callback(std::function<void (int)> callback);
%ignore register_temperature_changed_callback(std::function<void (const fp32*)> callback);
%ignore register_count_changed_callback(std::function<void (int)> callback);
%ignore release_report_data_callback(bool clear_all);
%ignore release_report_location_callback(bool clear_all);
%ignore release_connect_changed_callback(bool clear_all);
%ignore release_state_changed_callback(bool clear_all);
%ignore release_mode_changed_callback(bool clear_all);
%ignore release_mtable_mtbrake_changed_callback(bool clear_all);
%ignore release_error_warn_changed_callback(bool clear_all);
%ignore release_cmdnum_changed_callback(bool clear_all);
%ignore release_temperature_changed_callback(bool clear_all);*/

%ignore set_position_aa(fp32 pose[6], bool is_tool_coord = false, bool relative = false, bool wait = false, fp32 timeout = NO_TIMEOUT);
%ignore set_servo_cartesian_aa(fp32 pose[6], bool is_tool_coord = false, bool relative = false);
%ignore robotiq_set_activate(bool wait = true, unsigned char ret_data[6] = NULL);
%ignore robotiq_set_activate(unsigned char ret_data[6] = NULL);
%ignore robotiq_set_position(unsigned char pos, bool wait = true, fp32 timeout = 5, unsigned char ret_data[6] = NULL);
%ignore robotiq_set_position(unsigned char pos, bool wait = true, unsigned char ret_data[6] = NULL);
%ignore robotiq_set_position(unsigned char pos, unsigned char ret_data[6] = NULL);
%ignore robotiq_open(bool wait = true, fp32 timeout = 5, unsigned char ret_data[6] = NULL);
%ignore robotiq_open(bool wait = true, unsigned char ret_data[6] = NULL);
%ignore robotiq_open(unsigned char ret_data[6] = NULL);
%ignore robotiq_close(unsigned char speed = 0xFF, unsigned char force = 0xFF, bool wait = true, fp32 timeout = 5, unsigned char ret_data[6] = NULL);
%ignore robotiq_close(bool wait = true, fp32 timeout = 5, unsigned char ret_data[6] = NULL);
%ignore robotiq_close(bool wait = true, unsigned char ret_data[6] = NULL);
%ignore robotiq_close(unsigned char ret_data[6] = NULL);
%ignore open_bio_gripper(bool wait = true, fp32 timeout = 5);
%ignore close_bio_gripper(bool wait = true, fp32 timeout = 5);


//2nd solution for overloaded functions: renaming => TODO: not working
//%rename(set_position_short) XArmAPI::set_position(fp32 pose[6], fp32 radius, bool wait, fp32 timeout);  // Only rename in class XArmAPI
//%rename(set_position_arc) XArmAPI::set_position(fp32 pose[6], bool wait = false, fp32 timeout = NO_TIMEOUT);
%include "../lib/xArm-CPLUS-SDK/include/xarm/wrapper/xarm_api_simple.h"


