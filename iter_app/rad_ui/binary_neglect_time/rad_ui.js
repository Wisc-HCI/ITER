
// define update functions

function updateColor(seconds) {

  var GREEN = 127;
  var RED = 0;
  var LOWER_BOUND = 15;
  var UPPER_BOUND = 60;

  var scale;
  if (seconds <= LOWER_BOUND) {
    scale = RED;
  } else {
    scale = GREEN;
  }
  var color = 'hsl(' + Math.round(scale) + ', 75%, 50%)';

  $('#dot-'+i).css('background-color',color);

}

// Setup ROS Subscribers

var ros = new ROSLIB.Ros({
  url : 'ws://localhost:9090'
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

var listenerNegelectTime = new ROSLIB.Topic({
  ros: ros,
  name: '/rad/neglect_time',
  messageType: 'iter_app/TimeInterval'
});

listenerNegelectTime.subscribe(function(message) {
  if(message != undefined) {
    updateColor(message.current);
  }
});
