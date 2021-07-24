var mylib = require("../../swig/build/Release/xarm");

var xarm = new mylib.XArmAPI("130.82.171.9");

console.log('>Motion enable - Response: %i', xarm.motion_enable(true));
console.log('>Set mode 0 - Response: %i', xarm.set_mode(0));
console.log('>Set state 0 - Response: %i', xarm.set_state(0));


state_p = mylib.new_intp();
console.log('>Get state - Response: %i', xarm.get_state(state_p));
console.log('State: %i', mylib.intp_value(state_p));

console.log('>Set stop state (4) - Response: %i', xarm.set_state(4));
console.log('>Get state - Response: %i', xarm.get_state(state_p));
console.log('State: %i', mylib.intp_value(state_p));
