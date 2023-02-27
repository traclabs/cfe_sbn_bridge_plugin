#!/usr/bin/env node

//const { spawn } = require('node:child_process');
import * as child from 'child_process';
import yargs from 'yargs';
import { hideBin } from 'yargs/helpers';
import {TuiDashboard} from './tui.js';
import {SequenceGenerator} from './plantuml.js';

const portMappings = { // TODO: make this configurable
    2234: 'cpu1',
    2235: 'cpu2',
    2236: 'ros'
};


const args = yargs(hideBin(process.argv))
    .option('tui', { type: 'boolean', description: 'Enable TUI output mode' } )
    .option('interface', {
        type: 'string',
        description: 'Interface for TShark to listen on',
        default: 'lo'
    })
      .option('seq', {
          type: 'string',
          description: 'Filename to generate a PlantUML Sequence diagram of message traffic'
      })
    .argv;

var cnts = {};
clearCnts();

var cols = [
    {title:'#',width:6},
    {title:'Src',width:6},
    {title:'Dst',width:6},
    {title:'Type',width:10},
    {title:'Size',width:6},
    {title:'Proc',width:6},
    {title:'SC',width:6},
    {title:'Details',width:10}

];

var tui;
if (args.tui) {
    tui = new TuiDashboard(cols, cnts);
    tui.screen.key(['r'], () => {
        clearCnts();
        tui.pkts = [];
    });
    tui.shutdown_cb = shutdown;
} else {
    console.log("Initializing tshark for SBN on interface ", args.interface);
}

var seq;
if (args.seq) {
    seq = new SequenceGenerator(Object.values(portMappings), args.seq);
}

function shutdown() { // Callback function for clean exit
    if (seq) seq.close();
    cmd.kill('SIGINT');
}

const cmd = child.spawn('unbuffer', // unbuffer prevents caching so output is shown in real-time
                  ['tshark',
                   '-ni', args.interface,
                   '-Y', "sbn && not icmp",
                   '-T', 'fields',
                   '-e',  'frame.number',
                   '-e',  'udp.srcport',
                   '-e',  'udp.dstport',
                   '-e',  'sbn.message_type',
                   '-e',  'sbn.message_size',
                   '-e',  'sbn.processor_id',
                   '-e',  'sbn.spacecraft_id',
                   '-e',  'sbn.cfe_mid'
                  ]);

var lineBuffer = '';
var firstData = true;
cmd.stdout.on('data',  data => {
    // Split Lines
    var lines = (lineBuffer + data).split("\n");
    if (data[data.length-1] != '\n') {
        lineBuffer = lines.pop();
    } else {
        lineBuffer = '';
    }
    if (firstData) {
        firstData = false;
        lines.shift();
    }

    // Process single line
    for (const line of lines) {
        var cols = line.split('\t');

        // Display formatting conversions
        if (portMappings[cols[1]]) cols[1] = portMappings[cols[1]];
        if (portMappings[cols[2]]) cols[2] = portMappings[cols[2]];

        var details = '_';
        switch(parseInt(cols[3])) { // messageType to name conversion (incomplete)
        case 1:  cols[3] = "SUB(1)";   cnts.sub++;   break;
        case 2:  cols[3] = "UNSUB(2)"; cnts.unsub++; break;
        case 3:  cols[3] = "MSG(3)";   cnts.msg++; details=cols[7];  break;
        case 4:  cols[3] = "PROTO(4)"; cnts.proto++; break;            
        case 0xA0: cols[3]="HB(A0)";   cnts.hb++;    break;
        case 0xA1: cols[3]="ANC(A1)";  cnts.anc++;   break;
        case 0xA2: cols[3]="DIS(A2)";  cnts.dis++;   break;
        default: cols[3]="?("+cols[3]+")"; cnts.other++; break;
        }
        
        cols[5] = parseInt(cols[5],16);
        cols[6] = parseInt(cols[6],16);

        
        if (seq) seq.add(cols[1], cols[2], cols[3]);

        var disp_cols = [cols[0], cols[1], cols[2], cols[3], cols[4], cols[5], cols[6], details];
            
        if (args.tui) {
            tui.logPkt(disp_cols);
        } else {
            console.log(disp_cols.map(d => d.toString().padEnd(6)).join(" ") );
        }
    }
    if (tui) tui.refresh();

});
cmd.stderr.on('data', line => {
    console.log("STDERR:", line.toString());
});
cmd.on('close', code => {
    console.log("tshark exited with code ", code);
});

// Initialize (or reset) counters)
function clearCnts() {
    cnts = {
        "sub" : 0,
        "unsub" : 0,
        "msg" : 0,
        "proto" : 0,
        "hb" : 0,
        "anc": 0,
        "dis": 0,
        "other" : 0,
    };
    if (tui) tui.cnts = cnts;
}
