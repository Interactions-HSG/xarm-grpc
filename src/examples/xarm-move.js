var mylib = require("../../swig/build/Release/xarm");
const { performance } = require("perf_hooks")

var xarm = new mylib.XArmAPI("130.82.171.9");

console.log('>Motion enable - Response: %i', xarm.motion_enable(true));
console.log('>Set mode 0 - Response: %i', xarm.set_mode(0));
console.log('>Set state 0 - Response: %i', xarm.set_state(0));

console.log('Default is radian: ', xarm.default_is_radian);

posArray = mylib.new_floatArray(6);

// time measurements (2 Options)
console.time('set_position');
var t0 = performance.now()

// Set position

mylib.floatArray_setitem(posArray, 0, 400.20);
mylib.floatArray_setitem(posArray, 1, 0);
mylib.floatArray_setitem(posArray, 2, 200);
mylib.floatArray_setitem(posArray, 3, 180);
mylib.floatArray_setitem(posArray, 4, 0);
mylib.floatArray_setitem(posArray, 5, 0);

console.log('>Set position - Response: %i', xarm.set_position(posArray, -1, true, -1));

// time measurements (2 Options)
var t1 = performance.now();
console.timeEnd('set_position');
console.log("set_position with performance.now: " + (t1 - t0) + " ms.");

//Get position

console.log('>Get position - Response: %i', xarm.get_position(posArray));
console.log('Position:');
console.log(' x[mm]: %f', mylib.floatArray_getitem(posArray, 0));
console.log(' y[mm]: %f', mylib.floatArray_getitem(posArray, 1));
console.log(' z[mm]: %f', mylib.floatArray_getitem(posArray, 2));
console.log(' roll[rad/°]: %f', mylib.floatArray_getitem(posArray, 3));
console.log(' pitch[rad/°]: %f', mylib.floatArray_getitem(posArray, 4));
console.log(' yaw[rad/°]: %f', mylib.floatArray_getitem(posArray, 5));
