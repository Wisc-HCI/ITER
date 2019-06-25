
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

function updateSize(seconds) {
  //bounded to one minute?
  if (seconds > 60) {
    seconds = 60;
  }

  // scale adjust to fit to page
  size = seconds * 0.75;

  size_str = '' + size + 'em';
  padding_str = 'calc(50% - ' + size / 2 + 'em)';

  $('#container-0').css('left',padding_str);
  $('#container-0').css('top',padding_str);

  $('#dot-0').css('height',size_str);
  $('#dot-0').css('width',size_str);
  $('#dot-0').css('left',padding_str);
  $('#dot-0').css('top',padding_str);
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
      updateSize(interval.current);
    }

  }
});
