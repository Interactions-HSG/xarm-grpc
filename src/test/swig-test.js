
var mylib = require("../../swig/test/build/Release/mylib")
var c = new mylib.MyClass(8)
c.sayHello()

var a = new mylib.Xarm("port", 4, false);
console.log("Number: %i", a.myNumber)
console.log("Port: %s", a.myPort)
console.log("Bool: ", a.myBool)

