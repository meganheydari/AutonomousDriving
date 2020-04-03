// ===============================================
//  Created on 12/09/19.
//  Copyright © 2018 Chase Clarke, Megan Heydari, Leila Lee. All rights reserved.
// ===============================================


// ===============================================
// ============= SOCKETS =========================
// ===============================================
var app = require('express')();
var server = require('http').Server(app);
var io = require('socket.io')(server);

server.listen(80);

app.get('/', function (req, res) {
  res.sendFile(__dirname + '/static/index.html')
  
});



const recieveMode = require('socket.io').listen(process.env.port||5000);

recieveMode.on('connection', function(socket) {
    socket.on('data',function (data) {
        var parsed = JSON.parse(data)
        console.log(`Mode: '${data}'`)
        sendMode(parsed.mode)
    });
});

const recieveDirection = require('socket.io').listen(process.env.port||5001);
const keyToFunc = {[0] : start, [1] : leftTurn,[2] : rightTurn, [4] : scanQR};

recieveDirection.on('connection', function(socket) {
    socket.on('data',function (data) {
        var parsed = JSON.parse(data)
        console.log(parsed);
        keyToFunc[parsed.key](parsed.pressed);
    });
});


// ============= END SOCKETS ===================

// triggers qr code scanning from camera
function scanQR() {
  console.log('scan QR');
  var qrVal = ""

  var sys = require('sys')
  var exec = require('child_process').exec;
  function puts(error, stdout, stderr) {console.log(stdout);}
  exec("wget http://localhost:8080/?action=snapshot -0 ./QRjpg", puts);

  function puts(error, stdout, stderr) {console.log(stdout);}//io.emit('time', stdout.slice(8, stdout.length));}
  exec("zbarimg \"./QR.jpg\"", puts);

}

// left turn
function leftTurn(pressed) {
  if (pressed) {
    console.log("left start")
    sendToEsp("LEFT_START", "CRAWLER_INIT")
  }
  else {
    console.log("left stop")
    sendToEsp("LEFT_STOP", "CRAWLER_INIT")
  }
}

// right turn
function rightTurn(pressed) {
  if (pressed) {
    console.log('right start');
    sendToEsp("RIGHT_START", "CRAWLER_INIT")
  }
  else {
    console.log('right stop');
    sendToEsp("RIGHT_STOP", "CRAWLER_INIT")
  }
  

}

// starts crawler
function start(pressed) {
  if (pressed) {
    console.log('start movement');
    sendToEsp("START", "CRAWLER_INIT")
  }
  else {
    console.log('stop movement');
    sendToEsp("STOP", "CRAWLER_INIT")
  }
}

function sendMode(mode) {
  sendToEsp(mode.toString(), "CRAWLER_INIT")
}

function sendLapTime(laptime){
    sendToEsp(laptime.toString(), "HUB_INIT");
}



// ===============================================
// ================= UDP  ========================
// ===============================================

// Create socket
var dgram = require('dgram');
var server = dgram.createSocket('udp4');
const querystring = require('querystring');

var HOST = '192.168.1.108';
var UDPPORT = 8080;

ESPCLIENTS = { HUB_INIT : {ip : "", port : ""},
               CRAWLER_INIT : {ip : "", port : ""}};
// var portToEsp = []; // create an empty array


// Create server for key fob to connect to
server.on('listening', function () {
    var address = server.address();
    console.log('Initializing UDP Server on ' + address.address + ":" + address.port + "\n\n");
});

// on key fob connection save its ip:port
server.on('message', function (message, remote) {
    try {
      ESPCLIENTS[message].port = remote.port
      ESPCLIENTS[message].ip = remote.address

      console.log(message + '\'s fob connected and initialized esp @ ' + remote.address + ':' + remote.port)
    }
    catch {
      console.log("message from ESP: "+ message[0] + message[1], message[2], message[3])
      getSplit([message[0], message[1], message[2], message[3]])
      console.log("[!!!] ", message[1])
      if (message[1] == 71) {
        console.log("green")
        start(1);
      }
      else if (message[1] == 82) {
        console.log("red")

        start(0);
      }
    }
});

// function to send data back to esp after initial connection
function sendToEsp(serialData, name) {

    try {

      var port = ESPCLIENTS[name].port
      var host = ESPCLIENTS[name].ip

      // error checking
      if (port == "" || host == "") {
        console.log("fob is not connected to udp server")
        throw "err"
      }

      //send message to esp
      server.send(serialData, port, host,function(error){
          if(error) {
            console.log('ERROR: Sending ' + serialData + ' Signal to ESP.');
          }
          else {
            console.log('- Sending ' + serialData + ' to ESP: ' + port + ':' + host);
          }
        });
   

    }
    catch {
      console.log("no one with the name: "+ name + " connected to udp server")
    }
}

// split time testing function
// var beacon = 0
// function myTimer() {
//     beacon += 1
//     console.log('[!!!] ', beacon);
//     getSplit([27, 2, beacon, 27])

//     if (beacon == 3){
//       beacon = 0
//     }
// }
// var myVar = setInterval(myTimer, 3000);

function getSplit(data) {
  var a =  {"color" : data[1], "id" : data[2], "date" : Date.now()}
  // console.log("[!!!!]",a)
  dbAdd(a)
}

server.bind(UDPPORT, HOST);

// ============= END UDP ===================



// ===============================================
// ============= DATABASE CALLS ==================
// ===============================================
var dgram = require('dgram');
var Engine = require('tingodb')(),
    assert = require('assert');

// jsonObject = [{key : value, key : value, ...}]
function dbAdd(jsonObject) {
  var date = Date.now();

  db = new Engine.Db('.', {});
  collection = db.collection("usersDb");

  collection.insert([jsonObject], {w:1}, function(err, result) {
    //assert.equal(null, err);
    if (err) {console.log("dbadd error")}
      db.close();
  });
  
  if (jsonObject["id"] == 2){
    console.log("db get id 1")
    dbGet([{"id" : 1}], jsonObject["date"])
  }
  else if (jsonObject["id"] == 3){
    console.log("db get id 2")
    dbGet([{"id" : 2}], jsonObject["date"])

  }
  }

var tempSplitTime = 0
function dbGet(jsonObject, date) {
  db = new Engine.Db('.', {});
  collection = db.collection("usersDb");
  
  collection.find(jsonObject, {}).sort({_id:-1}).limit(1).toArray(function(err, result){
    dbCall = result;
    // console.log(result);
    // // console.log(result.length);
    try {
    var splitTime = (date - result[0]["date"])/1000
  }
  catch {
    console.log("error")
  }
    sendLapTime(Math.round(splitTime))

    if (splitTime > tempSplitTime) {
      io.emit('time', spliTime.toString())
    }

    tempSplitTime = splitTime
    db.close();
    });
}


// ========== END DATABASE CALLS =================
