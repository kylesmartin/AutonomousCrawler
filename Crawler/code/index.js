//By Kyle Martin, David Kirk, Ayush Upneja

//Socket and server
var express = require('express');
var app = express();
var http = require('http').createServer(app);
var io = require('socket.io')(http);
var dgram = require('dgram');
var server = dgram.createSocket('udp4');

// File reader
const fs = require('fs');
const readline = require('readline');
var count = 0;
var header;

// Port and IP of Host
var PORT = 3333; // external is 3333
var HOST = "192.168.1.122"; // David's laptop is .101, Kyle's Laptop is .102, Pi is .122, Ayush's Laptop is .142
const code = "smartkey";

//Port and IP of Device
var devPORT = 3333;
var devHOST = "192.168.1.100"

// TingoDB
var Engine = require('tingodb')(),
    assert = require('assert');

// Create DB
var db = new Engine.Db('.', {});
var users = db.collection("users_v4");
var logs = db.collection("logs");

var dirs = [0, 0, 1];

// Publish HTML file
app.get('/', function(req, res){
  res.sendFile(__dirname + '/main.html');
});

app.get('/logs', function(req, res){
   logs.find().toArray(function(err, result) {
     if (err) throw err;
     res.send(result);
   });
});

app.get('/params', function(req, res) {

  var speed = parseInt(req.param('speed'));
  var steer = parseInt(req.param('steer'));
  var start = parseInt(req.param('start'));
  console.log(start);

  res.send("response");

  dirs[0] += speed;

  if (Math.abs(dirs[1] + steer) <= 3) {
    dirs[1] += steer;
  }

  if (start) {
    console.log("mep");
    dirs[2] = 1 - dirs[2];
  }



  //console.log("speed: " + dirs[0] + "steer: " + dirs[1]);

  //console.log("speed: " + req);
  var dataStr = dirs[0].toString() + " " + dirs[1].toString() + " " + dirs[2].toString();
  console.log(dataStr + " - " + devHOST + ":" + devPORT);
  server.send(dataStr, devPORT, devHOST, function(error){});
});

// Send sensor readings to frontend and write JSON to local file
server.on('message', function (message, remote) {

  var splitTime = parseFloat(message);

  devPORT = remote.port;
  devHOST = remote.address;

  console.log(devHOST + ":" + devPORT + " time: " + splitTime);

  if (splitTime != NaN) {
    var d = new Date();
    var timeData = {"timestamp": d.toLocaleString(), "splitTime": splitTime};
    logs.insert(timeData);
  }

  /*
  // Parse message into JSON object
  var request = JSON.parse(message.toString());
  // Update device port and host

  // Query the fobID in the user database
  users.findOne({"fobID": request["fob_id"]}, function(err, item) {
    // if fobID and code are valid
    if (request["code"] == code && item != null) {
      server.send("granted",devPORT,devHOST,function(error){});
      var d = new Date();
      var info = {"name": item["name"], "fobID": request["fob_id"], "hubID": request["hub_id"], "timestamp": d.toLocaleString(), "access": "granted"};
      // log new key access request
      logs.insert(info);
      console.log(info);
      io.emit("message", info);
    // if fobID and code are invalid
    } else {
      server.send("denied",devPORT,devHOST,function(error){});
      var d = new Date();
      if (item == null) {
        var info = {"name": "unknown", "fobID": request["fob_id"], "hubID": request["hub_id"], "timestamp": d.toLocaleString(), "access": "denied"};
      } else {
        var info = {"name": item["name"], "fobID": request["fob_id"], "hubID": request["hub_id"], "timestamp": d.toLocaleString(), "access": "denied"};
      }
      // log new key access request
      logs.insert(info);
      console.log(info);
      io.emit("message", info);
    }
  })
  */
});

// User socket connection
io.on('connection', function(socket){
  console.log('a user connected');
  socket.on('disconnect', function(){
    console.log('user disconnected');
  });
});

// Listening on port 3000
http.listen(3000, function() {
  console.log('listening on *:3000');
});

// Create server
server.on('listening', function () {
    var address = server.address();
    console.log('UDP Server listening on ' + address.address + ":" + address.port);
});

// Bind server to port and IP
server.bind(PORT, HOST);
