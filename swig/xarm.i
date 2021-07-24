%module "xarm"
%{
#include "../lib/xArm-CPLUS-SDK/include/xarm/wrapper/xarm_api.h"
%}
// needed to support std::string types
%include "std_string.i"

// ===== Overloaded Functions Handling =====
// Option 1: just ignore
%ignore set_position(fp32 pose[6], fp32 radius = -1, fp32 speed = 0, fp32 acc = 0, fp32 mvtime = 0, bool wait = false, fp32 timeout = NO_TIMEOUT);
//%ignore set_position(fp32 pose[6], fp32 radius = -1, bool wait = false, fp32 timeout = NO_TIMEOUT);
%ignore set_position(fp32 pose[6], bool wait = false, fp32 timeout = NO_TIMEOUT);

%ignore set_tool_position(fp32 pose[6], bool wait = false, fp32 timeout = NO_TIMEOUT);

%ignore set_servo_angle(fp32 angles[7], bool wait = false, fp32 timeout = NO_TIMEOUT, fp32 radius = -1);
%ignore set_servo_angle(int servo_id, fp32 angle, fp32 speed = 0, fp32 acc = 0, fp32 mvtime = 0, bool wait = false, fp32 timeout = NO_TIMEOUT, fp32 radius = -1);
%ignore set_servo_angle(int servo_id, fp32 angle, bool wait = false, fp32 timeout = NO_TIMEOUT, fp32 radius = -1);

%ignore move_gohome(bool wait = false, fp32 timeout = NO_TIMEOUT);

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

//Option 2: renaming => TODO: not working
//%rename(set_position_short) XArmAPI::set_position(fp32 pose[6], fp32 radius, bool wait, fp32 timeout);  // Only rename in class XArmAPI
//%rename(set_position_arc) XArmAPI::set_position(fp32 pose[6], bool wait = false, fp32 timeout = NO_TIMEOUT);

// ===== Pointer Handling =====
// Option 1: using pointer library 
%include <cpointer.i>
%pointer_functions(int, intp);

// Option 2: using typemaps (currently only works for simple INPUT cases)
//%include <typemaps.i>
//%apply int *INOUT {int *state};
//int XArmAPI::get_state(int *INOUT);


// ===== Array Handling =====
// Option 1: using carrays library
%include "carrays.i"
%array_functions(float, floatArray);

// Option 2: using arrays (not working..)
/* array handling using: https://github.com/swig/swig/blob/master/Lib/javascript/v8/arrays_javascript.i
   TODO: according to source: "The typemaps are not efficient as there is a lot of copying of the array values 
   whenever the array is passed to C/C++ from JavaScript and vice versa."
   => check latency
*/
//%include <arrays_javascript.i>

// ===== Char[] Handling =====
// TODO: not working
//%include <cstring.i>
//%cstring_bounded_output(char *version, 40); => not implemented in js
%apply char * { unsigned char version[40] };

%include "../lib/xArm-CPLUS-SDK/include/xarm/wrapper/xarm_api.h"


