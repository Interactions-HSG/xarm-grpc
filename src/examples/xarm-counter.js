var mylib = require("../../swig/build/Release/xarm");

var xarm = new mylib.XArmAPI("130.82.171.9");


console.log('>Motion enable - Response: %i', xarm.motion_enable(true));
console.log('>Set mode 0 - Response: %i', xarm.set_mode(0));
console.log('>Set state 0 - Response: %i', xarm.set_state(0));

console.log('Counter: %i', xarm.count);
console.log('>Reset counter - Response: %i', xarm.set_counter_reset());
console.log('Counter: %i', xarm.count);
console.log('>Increase counter - Response: %i', xarm.set_counter_increase());
console.log('Counter: %i', xarm.count);

xarm.disconnect();
console.log('xArm is connected:', xarm.is_connected());
