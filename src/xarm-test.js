
var mylib = require("../swig/build/Release/xarm");

var xarm = new mylib.XArmAPI("130.82.171.9");
console.log('xArm state: %i', xarm.state);
console.log('xArm mode: %i', xarm.mode);

//console.log('xArm Version: %i', xarm.version_number); //TODO: check how to use pointers (typemaps)
//xarm.connect("130.82.171.9");

console.log('xArm is connected:', xarm.is_connected());
console.log('xArm has error:', xarm.has_error());
console.log('xArm has warn:', xarm.has_warn());

// console.log('xArm counter: %i', xarm.count);
// xarm.set_counter_increase();
// console.log('xArm counter: %i', xarm.count);
//xarm.motion_enable(true);
//xarm.set_mode(0);
//xarm.set_state(0);
//var version = '';
//xarm.get_version(version); // TODO: check how to use char[] => typemap

xarm.disconnect();
console.log('xArm is connected:', xarm.is_connected());

