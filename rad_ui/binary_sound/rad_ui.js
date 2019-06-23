//Warning sound = https://freesound.org/people/SamsterBirdies/sounds/467882/

// define constants

var GREEN = 127;
var RED = 0;

var NEGLECT_LOWER_BOUND = 15;

var AUDIO = new Audio('./samsterbirdies__beep-warning.mp3');

// define update functions

function updateColor(seconds) {

  var scale;
  if (seconds <= NEGLECT_LOWER_BOUND) {
    scale = RED;
    playWarning();
  } else {
    scale = GREEN;
  }
  var color = 'hsl(' + Math.round(scale) + ', 75%, 50%)';

  $('#dot-'+i).css('background-color',color);

}

var playedWarning = false;
function playWarning() {
  if (!playedWarning) {
    AUDIO.play();
    playedWarning = true;
  }
}

function modeChanged() {
  playedWarning = false;
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

    if (currentMode != message.mode) {
      currentMode = message.mode;
      modeChanged();
    }

    if (message.mode == 0) { // neglect time
      interval = message.neglect_time;
      updateColor(interval.current);
    }

  }
});
