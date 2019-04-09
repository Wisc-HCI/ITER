
// define update functions

function pad(num, size) {
    var s = num+"";
    while (s.length < size) s = "0" + s;
    return s;
}

function formatTime(secondsRaw) {
  var hours = Math.floor(secondsRaw / 3600);
  secondsRaw -= hours * 3600;

  var minutes = Math.floor(secondsRaw / 60);
  secondsRaw -= minutes * 60;

  var seconds = Math.floor(secondsRaw);

  var timeStr = pad(hours,2) + ':'
              + pad(minutes,2) + ':'
              + pad(seconds,2);
  return timeStr;
}

function updateProgressBar(value) {
  $('#rad-1-progressbar').css('width',(value * 100) + '%');
}

function updateColor(seconds) {

  var GREEN = 127;
  var RED = 0;
  var LOWER_BOUND = 15;
  var UPPER_BOUND = 60;

  var scale;

  if (seconds <= LOWER_BOUND) {
    scale = RED;
  } else if (seconds >= UPPER_BOUND) {
    scale = GREEN;
  } else {
    scale = (GREEN - RED) / (UPPER_BOUND - LOWER_BOUND) * (seconds - LOWER_BOUND) + RED;
  }

  color = 'hsl(' + Math.round(scale) + ', 75%, 50%)';

  $('#rad-1-progressbar').css('background-color',color);
}

function updateUpperBound(seconds) {
  $('#upper-bound').html(formatTime(seconds));
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
  messageType: 'iter_app/NeglectTime'
});

listenerNegelectTime.subscribe(function(message) {
  if(message != undefined) {
    updateColor(message.current);
    updateUpperBound(message.initial);
    updateProgressBar(message.current/message.initial);
  }
});
