# xarm-commander
A Node CLI Tool for controlling the xArm7 Robot using the [xArm-C++-SDK](https://github.com/xArm-Developer/xArm-CPLUS-SDK).
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
