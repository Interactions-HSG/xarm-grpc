const util = require('util');
const execFilePromise = util.promisify(require('child_process').execFile);

// read IP address of xArm control box
const IPaddress = process.argv[2];

// ===== using async + await =====

async function sendCommand(args) {
    const { stdout } = await execFilePromise('xarm-commander', args);
    console.log(stdout);
}

async function routine() {
    // sequential execution
    await sendCommand(['-i', IPaddress, '-v', 'motion_enable', '--enable']);
    await sendCommand(['-i', IPaddress, '-v', 'set_mode', '-m', '0']);
    await sendCommand(['-i', IPaddress, '-v', 'set_state', '-s', '0']);
    await sendCommand(['-i', IPaddress, '-v', 'get_version']);
    await sendCommand(['-i', IPaddress, '-v', 'get_state']);
    await sendCommand(['-i', IPaddress, '-v', 'get_position']);
    // parallel execution
    let res1 = sendCommand(['-i', IPaddress, '-v', 'get_version']);
    let res2 = sendCommand(['-i', IPaddress, '-v', 'get_state']);
    let res3 = sendCommand(['-i', IPaddress, '-v', 'get_position']);
    await res1
    await res2
    await res3

    // movement!
    // sequentially
    // await sendCommand(['-i', IPaddress, '-v', 'set_position', '-y', '100']);
    // await sendCommand(['-i', IPaddress, '-v', 'set_position', '-y', '-100']);
    // await sendCommand(['-i', IPaddress, '-v', 'set_position', '-y', '0']);
    // // parallel
    // res1 = sendCommand(['-i', IPaddress, '-v', 'set_position', '-y', '100']);
    // res2 = sendCommand(['-i', IPaddress, '-v', 'set_position', '-y', '-100']);
    // res3 = sendCommand(['-i', IPaddress, '-v', 'set_position', '-y', '0']);
    // await res1
    // await res2
    // await res3

    console.log("Routine Done!");
}

routine();
