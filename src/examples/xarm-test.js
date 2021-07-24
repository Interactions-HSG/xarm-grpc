
var mylib = require("../../swig/build/Release/xarm");

var xarm = new mylib.XArmAPI("130.82.171.9");
var res = 1;

console.log('----- member variable -----');

console.log('xArm state: %i', xarm.state);
console.log('xArm mode: %i', xarm.mode);
//console.log('xArm Version: %s', xarm.version_number); //TODO: check how to use pointers (typemaps)

console.log('----- methods test -----');

console.log('xArm is connected:', xarm.is_connected());
console.log('xArm has error:', xarm.has_error());
console.log('xArm has warn:', xarm.has_warn());


// pointer test for method: int get_state(int *num)
state_p = mylib.new_intp();
//mylib.intp_assign(state_p, 33);
//console.log(" state = " + mylib.intp_value(state_p));
res = xarm.get_state(state_p);
console.log('State: %i', mylib.intp_value(state_p));
console.log('Response: %i', res);

// array test for method: int get_position(fp32 pose[6])
posArray = mylib.new_floatArray(6);
res = xarm.get_position(posArray);
console.log('Position:');
console.log(' x[mm]: %f', mylib.floatArray_getitem(posArray, 0));
console.log(' y[mm]: %f', mylib.floatArray_getitem(posArray, 1));
console.log(' z[mm]: %f', mylib.floatArray_getitem(posArray, 2));
console.log(' roll[rad/°]: %f', mylib.floatArray_getitem(posArray, 3));
console.log(' pitch[rad/°]: %f', mylib.floatArray_getitem(posArray, 4));
console.log(' yaw[rad/°]: %f', mylib.floatArray_getitem(posArray, 5));
console.log('Response: %i', res);

// char[] test for method: int get_version(unsigned char version[40]);
//var version = "test";
// res = xarm.get_version(version);
// console.log('Version: ', version);
// console.log('Response: %i', res);

xarm.disconnect();
console.log('xArm is connected:', xarm.is_connected());

