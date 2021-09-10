
const util = require('util');
const execFilePromise = util.promisify(require('child_process').execFile);

const IPaddress = "130.82.171.9" // xarm7-2 

// ===== using promise =====

function sendCommand(args) {
    return new Promise((resolve, reject) => {
        // send command
        execFilePromise('xarm-commander', args)
            .then(({ stdout }) => {
                console.log(stdout);
                resolve()
            })
            // error handling TODO(jo-bru): proper error handling
            .catch((err) => {
                console.log("Error catched: " + err)
                reject()
            });

    });
}

// start routine
console.time("Timer-Routine");
sendCommand(['-i', IPaddress, '-v', 'motion_enable', '--enable'])
    .then(() => {
        console.timeLog("Timer-Routine");
        return sendCommand(['-i', IPaddress, '-v', 'set_mode', '-m', '0']);
    })
    .then(() => {
        console.timeLog("Timer-Routine");
        return sendCommand(['-i', IPaddress, '-v', 'set_state', '-s', '0']);
    })
    // sequentially
    .then(() => {
        console.time("Timer-Sync")
        console.timeLog("Timer-Routine");
        return sendCommand(['-i', IPaddress, '-v', 'get_version']);
    })
    .then(() => {
        console.timeLog("Timer-Routine");
        return sendCommand(['-i', IPaddress, '-v', 'get_state']);
    })
    .then(() => {
        console.timeLog("Timer-Routine");
        return sendCommand(['-i', IPaddress, '-v', 'get_position']);
    })
    // parallel
    .then(() => {
        console.timeEnd("Timer-Sync");
        console.time("Timer-Async");
        console.timeLog("Timer-Routine");
        return Promise.all([
            sendCommand(['-i', IPaddress, '-v', 'get_version']),
            sendCommand(['-i', IPaddress, '-v', 'get_state']),
            sendCommand(['-i', IPaddress, '-v', 'get_position'])]);
    })
    // movements!
    // sequentially
    // .then(() => {
    //     return sendCommand(['-i', IPaddress, '-v', 'set_position', '-y', '100']);
    // })
    // .then(() => {
    //     return sendCommand(['-i', IPaddress, '-v', 'set_position', '-y', '-100']);
    // })
    // .then(() => {
    //     return sendCommand(['-i', IPaddress, '-v', 'set_position', '-y', '0']);
    // })
    // .then(() => {
    //     // parallel
    //     return Promise.all([
    //         sendCommand(['-i', IPaddress, '-v', 'set_position', '-y', '100']),
    //         sendCommand(['-i', IPaddress, '-v', 'set_position', '-y', '-100']),
    //         sendCommand(['-i', IPaddress, '-v', 'set_position', '-y', '0'])]);
    // })
    .then(() => {
        console.timeEnd("Timer-Async");
        console.timeEnd("Timer-Routine");
        console.log("Routine finished!");
    });









