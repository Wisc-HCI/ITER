
// define constants

var GREEN = 127;
var RED = 0;

var NEGLECT_LOWER_BOUND = 15;

// define update functions

function updateColor(seconds) {

  var scale;
  if (seconds <= NEGLECT_LOWER_BOUND) {
    scale = RED;
  } else {
    scale = GREEN;
  }
  var color = 'hsl(' + Math.round(scale) + ', 75%, 50%)';

  $('#dot-0').css('background-color',color);

}

// Setup ROS Subscribers
$.getJSON("../rosbridge_properties.json", function(json) {
  var ros = new ROSLIB.Ros({
    url : `ws://${json.host}:${json.port}`
  });

  ros.on('connection', function() {
    console.log('Connected to websocket server.');
  });

  ros.on('error', function(error) {
    console.log('Error connecting to websocket server: ', error);
  });

  ros.on('close', function() {
    console.log('Connection to websocket server closed.');
    alert('Connection to websocket server closed.');
  });

  var listenerRadSignal = new ROSLIB.Topic({
    ros: ros,
    name: '/rad/signal',
    messageType: 'iter_app/RADSignal'
  });

  var currentMode = -1;
  listenerRadSignal.subscribe(function(message) {
    if(message != undefined) {

      if (message.mode == 0) { // neglect time
        interval = message.neglect_time;
        updateColor(interval.current);
      }

    }
  });
});