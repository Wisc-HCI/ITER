
//==============================================================================
// global Variables
//==============================================================================

const COLOR_BLACK = '#000';
const COLOR_GREEN = '#4caf50';
const COLOR_BLUE = '#0097a7';
const COLOR_YELLOW = '#ffd54f';
const COLOR_RED = '#ef5350';
const COLOR_PURPLE = '#563D7C';
const COLOR_DARK_GRAY = '#757575';

const NEGLECT_LOWER_BOUND = 5;

//==============================================================================
// global functions
//==============================================================================

function pad(num, size) {
    let s = num+"";
    while (s.length < size) s = "0" + s;
    return s;
}

function formatTime(secondsRaw) {
  let hours = Math.floor(secondsRaw / 3600);
  secondsRaw -= hours * 3600;

  let minutes = Math.floor(secondsRaw / 60);
  secondsRaw -= minutes * 60;

  let seconds = Math.floor(secondsRaw);

  let timeStr = pad(minutes,2) + ':'
              + pad(seconds,2);
  return timeStr;
}

function updateTime(seconds) {
  $('#time-1-text').html(formatTime(seconds));
}

function updateColorBoxNeglect(seconds) {

  let color;
  if (seconds < 1) {
    color = COLOR_RED;
  } else if (seconds < NEGLECT_LOWER_BOUND) {
    color = COLOR_YELLOW;
  } else {
    color = COLOR_GREEN;
  }

  $('#color-box-1').css('background-color',color);
}

function updateColorBoxInteraction() {
  $('#color-box-1').css('background-color',COLOR_RED);
}

//==============================================================================
// Events
//==============================================================================

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

  listenerRadSignal.subscribe(function(message) {
    if(message != undefined) {
      if (message.mode == 0) { // neglect time
        let interval = message.neglect_time;
        updateTime(interval.current);
        updateColorBoxNeglect(interval.current);
      } else if (message.mode == 1) { // interaction time
        updateColorBoxInteraction();
      }
    }
  });
});
