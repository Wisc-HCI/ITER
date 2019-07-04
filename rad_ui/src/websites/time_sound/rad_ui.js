//Warning sound = https://freesound.org/people/SamsterBirdies/sounds/467882/

// define constants

var NEGLECT_LOWER_BOUND = 15;

var AUDIO = new Audio('./samsterbirdies__beep-warning.mp3');

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

function updateTime(seconds) {
  $('#time-1-text').html(formatTime(seconds));
}

function warningCheck(seconds) {
  if (seconds <= NEGLECT_LOWER_BOUND) {
    playWarning();
  }
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

      if (currentMode != message.mode) {
        currentMode = message.mode;
        modeChanged();
      }

      var interval;
      if (message.mode == 0) { // neglect time
        interval = message.neglect_time;
        warningCheck(interval.current);
        updateTime(interval.current);
      }

    }
  });
});
