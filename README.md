# xarm-commander
A Node CLI Tool for controlling the xArm7 Robot using the [xArm-C++-SDK](https://github.com/xArm-Developer/xArm-CPLUS-SDK).
This is only a demonstrational version, future versions will be implemented using C++.

## Run Commander
For demonstration purposes, only two simple commands have been implemented so far.  
To use the xarm-commander, first install the npm module (globally):
```
npm install -g
```
Use xarm-commander: (Enable the motion control of the xArm7.)
```
xarm-commander motion_enable true
```
> :bulb: the xArm7 is only accessible from within the interactions network!  

## Run Examples
More commands are implemented as simple .js scripts to show the usage for different  types of arguements and return values.  
To run an example script simple run:
```
node src/examples/xarm-move.js 
```

## Run Tests
By now, following simple examples are implemented to test the underlying frameworks: [SWIG](https://github.com/swig/swig) and [Commander.js](https://www.npmjs.com/package/commander).

### Swig Test
Example taken from: https://www.programmersought.com/article/3416819205/

First create a wrapper using SWIG: (inside `swig\test`)
```
swig -javascript -node -c++ mylib.i
```
Then run node-gyp build to actually create the module:
```
node-gyp build
```
> :bulb: on first time, use: `node-gyp configure`  
> to remove any generated build files and the "out" dir, use: `node-gyp clean`  
  
Then run actual test script:
```
node ../../src/test/swig-test.js
```
### Commander Test (command line tool)
Install xarm-commander module (globally):
```
npm install -g
```
Use xarm-commander:
```
xarm-commander -i 192.168.1.2 send "Hello World!"
```
