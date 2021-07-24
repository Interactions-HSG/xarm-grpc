#!/usr/bin/env node

var program = require('commander');
const options = program.opts();

var name = "Jonas";

program
    .version('0.0.1')
    .option('-i, --ip-address <ip>', 'IP address of xArm control box.', '192.168.0.0')

    .command('send <message>')
    .description('send message.')
    .action((message) => {
        console.log(`Hi ${name}, this is your first message: ${message} (${options.ipAddress})`);
    });


program.parse(process.argv);
