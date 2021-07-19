{
  "targets": [
    {
      "target_name": "xarm",
      "include_dirs":["../lib/xArm-CPLUS-SDK/include"],
      "sources": ["xarm_wrap.cxx",
                  # automatic file listing (not used)
                  #"<!@(node -p \"var fs=require('fs'),path=require('path'),walk=function(r){let t,e=[],n=null;try{t=fs.readdirSync(r)}catch(r){n=r.toString()}if(n)return n;var a=0;return function n(){var i=t[a++];if(! i)return e;let u=path.resolve(r,i);i=r+'/'+i;let c=fs.statSync(u);if(c&&c.isDirectory()){let r=walk(i);return e=e.concat(r),n()}return e.push(i),n()}()};walk('../lib/xArm-CPLUS-SDK/src').join(' ');\")"],
                  '../lib/xArm-CPLUS-SDK/src/serial/impl/list_ports/list_ports_linux.cc',
                  #'../lib/xArm-CPLUS-SDK/src/serial/impl/list_ports/list_ports_osx.cc',
                  #'../lib/xArm-CPLUS-SDK/src/serial/impl/list_ports/list_ports_win.cc',
                  '../lib/xArm-CPLUS-SDK/src/serial/impl/unix.cc',
                  #'../lib/xArm-CPLUS-SDK/src/serial/impl/win.cc',
                  '../lib/xArm-CPLUS-SDK/src/serial/serial.cc',
                  '../lib/xArm-CPLUS-SDK/src/xarm/core/common/crc16.cc',
                  '../lib/xArm-CPLUS-SDK/src/xarm/core/common/queue_memcpy.cc',
                  '../lib/xArm-CPLUS-SDK/src/xarm/core/connect.cc',
                  '../lib/xArm-CPLUS-SDK/src/xarm/core/debug/debug_print.cc',
                  '../lib/xArm-CPLUS-SDK/src/xarm/core/instruction/uxbus_cmd.cc',
                  '../lib/xArm-CPLUS-SDK/src/xarm/core/instruction/uxbus_cmd_ser.cc',
                  '../lib/xArm-CPLUS-SDK/src/xarm/core/instruction/uxbus_cmd_tcp.cc',
                  '../lib/xArm-CPLUS-SDK/src/xarm/core/os/bak/network_new.cc',
                  #'../lib/xArm-CPLUS-SDK/src/xarm/core/os/bak/network_old.cc',
                  #'../lib/xArm-CPLUS-SDK/src/xarm/core/os/bak/thread.cc',
                  #'../lib/xArm-CPLUS-SDK/src/xarm/core/os/network.cc',
                  '../lib/xArm-CPLUS-SDK/src/xarm/core/port/ser.cc',
                  '../lib/xArm-CPLUS-SDK/src/xarm/core/port/socket.cc',
                  '../lib/xArm-CPLUS-SDK/src/xarm/core/report_data.cc',
                  '../lib/xArm-CPLUS-SDK/src/xarm/wrapper/xarm_api.cc',
                  '../lib/xArm-CPLUS-SDK/src/xarm/wrapper/xarm_bio.cc',
                  '../lib/xArm-CPLUS-SDK/src/xarm/wrapper/xarm_events.cc',
                  '../lib/xArm-CPLUS-SDK/src/xarm/wrapper/xarm_gpio.cc',
                  '../lib/xArm-CPLUS-SDK/src/xarm/wrapper/xarm_gripper.cc',
                  '../lib/xArm-CPLUS-SDK/src/xarm/wrapper/xarm_motion.cc',
                  '../lib/xArm-CPLUS-SDK/src/xarm/wrapper/xarm_params.cc',
                  '../lib/xArm-CPLUS-SDK/src/xarm/wrapper/xarm_record.cc',
                  '../lib/xArm-CPLUS-SDK/src/xarm/wrapper/xarm_report.cc',
                  '../lib/xArm-CPLUS-SDK/src/xarm/wrapper/xarm_robotiq.cc',
                  '../lib/xArm-CPLUS-SDK/src/xarm/wrapper/xarm_servo.cc'],
      'cflags': ['-fexceptions'],
      'cflags_cc': ['-fexceptions']

    }
  ]
}

