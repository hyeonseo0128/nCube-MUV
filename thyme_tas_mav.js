var net = require('net');
var ip = require('ip');
var moment = require('moment');
var fs = require('fs');
var Gpio = require('onoff').Gpio; 

var pushButton = new Gpio(19, 'in', 'both'); 
var mavlink = require('./mavlibrary/mavlink.js');

var _server = null;

var socket_mav = null;
global.mavPort = null;

var mavPortNum = '/dev/ttyAMA0';
var mavBaudrate = '115200';
var my_drone_type = 'pixhawk';
tas_ready();

function tas_ready() {
    if ((my_drone_type === 'pixhawk') || (my_drone_type === 'ardupilot') || (my_drone_type === 'px4')) {
        mavPortNum = '/dev/ttyAMA0';
        mavBaudrate = '115200';
        mavPortOpening();
    } else {
    }
}

var SerialPort = require('serialport');

function mavPortOpening() {
    if (mavPort == null) {
        mavPort = new SerialPort(mavPortNum, {
            baudRate: parseInt(mavBaudrate, 10),
        });

        mavPort.on('open', mavPortOpen);
        mavPort.on('close', mavPortClose);
        mavPort.on('error', mavPortError);
        mavPort.on('data', mavPortData);
    } else {
        if (mavPort.isOpen) {

        } else {
            mavPort.open();
        }
    }
}

function mavPortOpen() {
    console.log('mavPort open. ' + mavPortNum + ' Data rate: ' + mavBaudrate);
}

function mavPortClose() {
    console.log('mavPort closed.');

    setTimeout(mavPortOpening, 2000);
}

function mavPortError(error) {
    var error_str = error.toString();
    console.log('[mavPort error]: ' + error.message);
    if (error_str.substring(0, 14) == "Error: Opening") {

    } else {
        console.log('mavPort error : ' + error);
    }

    setTimeout(mavPortOpening, 2000);
}

global.mav_ver = 1;

const byteToHex = [];

for (let n = 0; n <= 0xff; ++n) {
    const hexOctet = n.toString(16).padStart(2, "0");
    byteToHex.push(hexOctet);
}

var mavStrFromDrone = '';
var mavStrFromDroneLength = 0;

function mavPortData(data) {
    mavStrFromDrone += data.toString('hex').toLowerCase();
    while (mavStrFromDrone.length > 12) {
        var stx = mavStrFromDrone.substr(0, 2);
        if (stx === 'fe') {
            var len = parseInt(mavStrFromDrone.substr(2, 2), 16);
            var mavLength = (6 * 2) + (len * 2) + (2 * 2);

            if ((mavStrFromDrone.length) >= mavLength) {
                var mavPacket = mavStrFromDrone.substr(0, mavLength);
                setTimeout(parseMavFromDrone, 0, mavPacket);

                //if(mavStrFromDroneLength > 0) {
                mavStrFromDrone = mavStrFromDrone.substr(mavLength);
                mavStrFromDroneLength = 0;
                //}
            } else {
                break;
            }
        } else {
            mavStrFromDrone = mavStrFromDrone.substr(2);
            //console.log(mavStrFromDrone);
        }
    }
}

var fc = {};

fc.global_position_int = {};
fc.global_position_int.time_boot_ms = 123456789;
fc.global_position_int.lat = 0;
fc.global_position_int.lon = 0;
fc.global_position_int.alt = 0;
fc.global_position_int.vx = 0;
fc.global_position_int.vy = 0;
fc.global_position_int.vz = 0;
fc.global_position_int.hdg = 65535;


function parseMavFromDrone(mavPacket) {
    try {
        var ver = mavPacket.substr(0, 2);
        if (ver == 'fd') {
            var sysid = mavPacket.substr(10, 2).toLowerCase();
            var msgid = mavPacket.substr(14, 6).toLowerCase();
        } else {
            sysid = mavPacket.substr(6, 2).toLowerCase();
            msgid = mavPacket.substr(10, 2).toLowerCase();
        }

        var sys_id = parseInt(sysid, 16);
        var msg_id = parseInt(msgid, 16);

        var cur_seq = parseInt(mavPacket.substr(4, 2), 16);

        if (msg_id == mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT) { // #33
            if (ver == 'fd') {
                var base_offset = 20;
                var time_boot_ms = mavPacket.substr(base_offset, 8).toLowerCase();
                base_offset += 8;
                var lat = mavPacket.substr(base_offset, 8).toLowerCase();
                base_offset += 8;
                var lon = mavPacket.substr(base_offset, 8).toLowerCase();
                base_offset += 8;
                var alt = mavPacket.substr(base_offset, 8).toLowerCase();
                base_offset += 8;
                var relative_alt = mavPacket.substr(base_offset, 8).toLowerCase(1``);
            } else {
                base_offset = 12;
                time_boot_ms = mavPacket.substr(base_offset, 8).toLowerCase();
                base_offset += 8;
                lat = mavPacket.substr(base_offset, 8).toLowerCase();
                base_offset += 8;
                lon = mavPacket.substr(base_offset, 8).toLowerCase();
                base_offset += 8;
                alt = mavPacket.substr(base_offset, 8).toLowerCase();
                base_offset += 8;
                relative_alt = mavPacket.substr(base_offset, 8).toLowerCase();
            }

            fc.global_position_int.time_boot_ms = Buffer.from(time_boot_ms, 'hex').readUInt32LE(0);
            fc.global_position_int.lat = Buffer.from(lat, 'hex').readInt32LE(0);
            fc.global_position_int.lon = Buffer.from(lon, 'hex').readInt32LE(0);
            fc.global_position_int.alt = Buffer.from(alt, 'hex').readInt32LE(0);
            fc.global_position_int.relative_alt = Buffer.from(relative_alt, 'hex').readInt32LE(0);
            
            muv_mqtt_client.publish(muv_pub_fc_gpi_topic, JSON.stringify(fc.global_position_int));
        }
    } catch (e) {
        console.log('[parseMavFromDrone Error]', e.message);
    }
}

function saveFile() {
    fs.appendFile('./gps.txt', [fc.global_position_int.lat, fc.global_position_int.lon] + "\n", 'utf8', function (error) {
        if (error) {
            console.log(error)
        }
    });
}

pushButton.watch(function (err, value) { //Watch for hardware interrupts on pushButton GPIO, specify callback function
    if (err) { //if an error
        console.error('There was an error', err); //output error message to console
        return;
    }
    console.log("button: ",pushButton.readSync());
    if (pushButton.readSync() == 0) { //check the pin state, if the state is 0 (or off)
        saveFile()
    }
    else {
        saveFile()
    }
});

function unexportOnClose() { //function to run when exiting program
    LED.writeSync(0); // Turn LED off
    LED.unexport(); // Unexport LED GPIO to free resources
    pushButton.unexport(); // Unexport Button GPIO to free resources
};

process.on('SIGINT', unexportOnClose); //function to run when user closes using ctrl+c
