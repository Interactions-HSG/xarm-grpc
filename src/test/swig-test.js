
var mylib = require("../../swig/test/build/Release/mylib")
var c = new mylib.MyClass(8)
c.sayHello()

var a = new mylib.Xarm("port", 4, false);
console.log("Number: %i", a.myNumber)
console.log("Port: %s", a.myPort)
console.log("Bool: ", a.myBool)

// ===== Test Run: Pointers =====
console.log("Testing the pointer library");

num = mylib.new_intp();
mylib.intp_assign(num, 33);
console.log(" num = " + mylib.intp_value(num));

var status = 0;
status = a.run(num);
console.log('Number: %i', mylib.intp_value(num));
console.log('Status: %i', status);


// var num = 4;
// var response = false;
// //console.log('test: ', a.run());
// numb = a.run();
// //console.log('Number: %i', status);
// console.log('Status: %i', numb);

//var array = mylib.doubleArray(2);
//var array = mylib.static_array;
// array[0] = 1;
// array[1] = 5;
// console.log('Test array: %i, %i', array[0], array[1])
//ar array = [0, 0];
//array = mylib.array();
//console.log('Array : %i, %i', array[0], array[1])

