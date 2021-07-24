#!/usr/bin/env node

var program = require('commander');
var mylib = require("../swig/build/Release/xarm");

const options = program.opts();
var xarm = new mylib.XArmAPI("130.82.171.9");

program
    .version('0.0.1')
    //.option('-i, --ip-address <ip>', 'IP address of xArm control box.', '192.168.0.0')

    .command('motion_enable <bool>')
    .description('Enable motion control.')
    .action((bool) => {
        if (bool == 'true') {
            console.log('>Motion enable - Response: %i', xarm.motion_enable(true));
        } else {
            console.log('>Motion disable - Response: %i', xarm.motion_enable(false));
        }
    });


function myParseInt(value, dummyPrevious) {
    // parseInt takes a string and a radix
    const parsedValue = parseInt(value, 10);
    if (isNaN(parsedValue)) {
        throw new commander.InvalidArgumentError('Not a number.');
    }
    return parsedValue;
}

program
    .command('set_mode')
    .description('Set the xArm mode.')
    .argument('<mode_int>', 'Mode', myParseInt)
    .action((mode_int) => {
        console.log('>Set mode %i - Response: %i', mode_int, xarm.set_mode(mode_int));
    });


program.parse(process.argv);
