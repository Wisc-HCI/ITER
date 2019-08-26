
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

const NEGLECT_LOWER_BOUND = 15;

var timeline = null;
var times = null;

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

//==============================================================================
// Object Constructors
//==============================================================================

function Playhead(canvas, x, y) {
  let self = this;

  self.nested = canvas.nested();
  self.triangleTop = self.nested.polygon('0,0 20,0 10,20').fill(COLOR_BLACK);
  self.triangleBottom = self.nested.polygon('0,140 20,140 10,120').fill(COLOR_BLACK);
  self.line = self.nested.line(10,15,10,125).attr({
    stroke: COLOR_BLACK,
    'stroke-width': 3
  });
  self.circle = self.nested.circle(75).move(-29,-74).stroke({
    width: 3,
    color: COLOR_BLACK
  }).fill({
    color: COLOR_BLACK,
    opacity: 0.0
  });
  self.icon = self.nested.nested().dmove(-16,-60).size(50,50);

  self.dmove = function(x, y) {
    self.nested.dmove(x,y);
  };
  self.dmove(x,y);

  self.setNeglectIcon = function() {
    if (self._neglectIconString != null) {
      self.icon.clear();
      self.icon.svg(self._neglectIconString);
      self.circle.fill({
        color: COLOR_GREEN,
        opacity: 1.0
      });
    } else {
      self._asyncSetIcon('_neglectIconString',self.setNeglectIcon);
    }
  }

  self.setWarningIcon = function() {
    if (self._warningIconString != null) {
      self.icon.clear();
      self.icon.svg(self._warningIconString);
      self.circle.fill({
        color: COLOR_YELLOW,
        opacity: 1.0
      });
    } else {
      self._asyncSetIcon('_warningIconString',self.setWarningIcon);
    }

  }

  self.setInteractionIcon = function() {
    if (self._interactionIconString != null) {
      self.icon.clear();
      self.icon.svg(self._interactionIconString);
      self.circle.fill({
        color: COLOR_RED,
        opacity: 1.0
      });
    } else {
      self._asyncSetIcon('_interactionIconString',self.setInteractionIcon);
    }

  }

  self.setWaitingIcon = function() {
    if (self._waitingIconString != null) {
      self.icon.clear();
      self.icon.svg(self._waitingIconString);
      self.circle.fill({
        color: COLOR_BLACK,
        opacity: 0.0
      });
    } else {
      self._asyncSetIcon('_waitingIconString',self.setWaitingIcon);
    }
  }

  self._getIcon = function(file, attribute) {
    $.get(file, function(contents) {
      let tmp = new XMLSerializer().serializeToString(contents);
      self[attribute] = tmp;
    });
  }

  self._neglectIconString = null;
  self._warningIconString = null;
  self._interactionIconString = null;
  self._waitingIconString = null;

  self._getIcon('./icons/neglect.svg','_neglectIconString');
  self._getIcon('./icons/warning.svg','_warningIconString');
  self._getIcon('./icons/interaction.svg','_interactionIconString');
  self._getIcon('./icons/waiting.svg','_waitingIconString');

  self._cancelIntervalID = null
  self._asyncSetIcon = function(iconString, setFunction) {

    if (self._cancelIntervalID != null) {
      clearInterval(self._cancelIntervalID);
      self._cancelIntervalID = null;
    }

    let cancel = setInterval(function(){
      if (self[iconString] != null) {
        clearInterval(cancel);
        self._cancelIntervalID = null;
        setFunction();
      }
    }, 100);
    self._cancelIntervalID = cancel;
  }
  self.setWaitingIcon();

  self.x = function() {
    return self.nested.x();
  }
}

function Tile(canvas, x, y, width, timeInfo) {
  this.state = 'okay';
  this.timeInfo = timeInfo;

  let color = (this.timeInfo.type == 'interaction') ? COLOR_PURPLE : COLOR_BLUE;
  this.rectangle = canvas.rect(width,100).radius(5).fill(color);

  this.dmove = function(x, y) {
    this.rectangle.dmove(x,y);
  }
  this.dmove(x,y);

  this.extend = function(dx,dt) {
    this.rectangle.width(this.rectangle.width() + dx);
    this.timeInfo.stop_time += dt;
  }

  this.timeAdjust = function(dx,dt) {
    this.dmove(dx,0);
    this.timeInfo.start_time += dt;
    this.timeInfo.stop_time += dt;
  }

  this.setFinished = function() {
    this.rectangle.fill(COLOR_DARK_GRAY);
    this.state = 'finished';
  }
}

function Tick(canvas,x,y,major=false,time=0,label=true) {

  this._x = x;
  this.lastColor = COLOR_BLACK;
  this.lastStrokeWidth = 0.25

  this.nested = canvas.nested();
  if (major) {
    this.line = this.nested.line(x,y,x,y+20).attr({
      stroke: COLOR_BLACK,
      'stroke-width': 2
    });
  } else {
    this.line = this.nested.line(x,y,x,y+10).attr({
      stroke: COLOR_BLACK,
      'stroke-width': 1
    });
  }

  if (label) {
    this.label = this.nested.text(formatTime(time)).move(x-20,y+35).attr({'stroke-width': 0.25});
  }

  this.x = function() {
    return this._x;
  }

  this.dmove = function(dx,dy) {
    this.line.dmove(dx,dy);
    this.label.dmove(dx,dy);
    this._x += dx;
  }

  this.clear = function() {
    this.line = null;
    this.label = null;
    this.nested.clear();
  }

  this.setColor = function(color, strokeWidth=0.25) {
    if (this.lastColor != color || this.lastStrokeWidth != strokeWidth) {
      this.lastColor = color;
      this.lastStrokeWidth = strokeWidth;
      this.line.stroke(color);
      this.label.stroke(color).attr({'stroke-width': strokeWidth});
    }
  }
}

function Timeline(canvas, x_start, x_end, x_playhead, y, times) {
  let x_center = (x_end - x_start) / 2 + x_start;

  const TIME_STEP = 15;
  const X_STEP = 50;

  const TILE_Y_OFFSET = -50;
  const TICK_Y_OFFSET = 80;

  this.tiles = [];
  this.ticks = [];

  this.time = 0;
  this.tileIndex = 0;
  this.stop_time = (times.length > 0) ? times[times.length-1].stop_time : 0;

  this._drawTiles = function(start_x, y) {

    for (let t of times) {
      let width = (t.stop_time - t.start_time) * X_STEP / TIME_STEP;
      let x = start_x + t.start_time * X_STEP / TIME_STEP;
      this.tiles.push(new Tile(canvas, x, y, width, t));
    }
  };
  this._drawTiles(x_playhead,y+TILE_Y_OFFSET);

  this._drawTicks = function(start_x, start_time, stop_time, y) {
    let x = start_x;
    for (let t=start_time; t<stop_time+TIME_STEP; t+=TIME_STEP) {
      this.ticks.push(new Tick(canvas,x,y,(t % 60 == 0),t));
      x += X_STEP;
    }
  };
  this._drawTicks(x_playhead,0,this.stop_time,y+TICK_Y_OFFSET);

  this.playhead = new Playhead(canvas,x_playhead-10,y-70);
  this.updatePlayheadIcon = function() {
    console.log('Setting icon based on timeline data');
    if (this.tileIndex < this.tiles.length) {
      if (this.tiles[this.tileIndex].timeInfo.type == 'interaction') {
        this.playhead.setInteractionIcon();
      } else {
        this.playhead.setNeglectIcon();
      }
    } else {
      this.playhead.setWaitingIcon();
    }
  }
  this.updatePlayheadIcon();

  this.update = function(time, interacting) {
    dt = time - this.time
    dx = -dt * X_STEP / TIME_STEP;

    if (this.tileIndex < this.tiles.length) {

      activeTile = this.tiles[this.tileIndex];
      if (activeTile.timeInfo.type == 'interaction' && interacting && activeTile.timeInfo.stop_time <= this.time + dt) {
        // adjust tiles
        activeTile.extend(-dx,dt);
        for (let i=this.tileIndex+1; i<this.tiles.length; i++) {
          this.tiles[i].timeAdjust(-dx,dt);
        }

        this.stop_time += dt;

        // update tick marks
        let lastTick = this.ticks[this.ticks.length-1];
        this._drawTicks(lastTick.x() + X_STEP,this.ticks.length*TIME_STEP,this.stop_time+dt,y+TICK_Y_OFFSET);
        for (let i=0; i<this.ticks.length; i++) {
          if (i*TIME_STEP >= this.original_stop_time && this.stop_time >= this.original_stop_time) {
            this.ticks[i].setColor(COLOR_RED,1);
          } else if (i*TIME_STEP <= this.stop_time) {
            this.ticks[i].setColor(COLOR_BLACK);
          }
        }

        // recenter timeline
        this.recenter();

      } else if (activeTile.timeInfo.type == 'interaction' && !interacting) {
        let qt = time - activeTile.timeInfo.stop_time - 0.01;
        let qx = qt * X_STEP / TIME_STEP;

        // adjust tiles
        activeTile.extend(qx,qt);
        for (let i=this.tileIndex+1; i<this.tiles.length; i++) {
          this.tiles[i].timeAdjust(qx,qt);
        }
        this.stop_time += qt;

        // update tick marks
        for (let i=0; i<this.ticks.length; i++) {
          if (i*TIME_STEP >= this.stop_time) {
            if (i*TIME_STEP <= this.original_stop_time) {
              this.ticks[i].setColor(COLOR_GREEN,1);
            } else if ((i-1)*TIME_STEP >= this.stop_time) {
              this.ticks[i].clear();
              this.ticks.splice(i,1);
              i--; // step index back
            }
          }
        }

        // recenter timeline
        this.recenter();

      } else if (activeTile.timeInfo.type == 'neglect') {
        if (activeTile.state != 'warning' && activeTile.timeInfo.stop_time - (this.time+dt) < NEGLECT_LOWER_BOUND) {
          this.playhead.setWarningIcon();
        }
      }
    }

    // shift playhead and/or timeline
    if (this.playhead.x() < x_center) {     // move playhead
      this.move(0,-dx);
    } else if (this.length() < x_end) {     // move playhead
      this.move(0,-dx);
    } else {                                // move timeline
      this.move(dx,0);
    }

    // update timing and active tile
    this.time = time;
    while (this.tileIndex < this.tiles.length && this.time > this.tiles[this.tileIndex].timeInfo.stop_time) {
      this.tiles[this.tileIndex].setFinished();
      this.tileIndex++;
      this.updatePlayheadIcon();
    }
  }

  this.move = function(timeline_dx, playhead_dx) {
    // TODO handle playhead movement
    for (let t of this.tiles) {
      t.dmove(timeline_dx,0);
    }
    for (let t of this.ticks) {
      t.dmove(timeline_dx,0);
    }

    this.playhead.dmove(playhead_dx,0);
  }

  this.length = function() {
    if (this.ticks.length > 0) {
      let lastTick = this.ticks[this.ticks.length-1];
      return lastTick.x();
    } else {
      return x_start;
    }
  }

  this.start = function() {
    if (this.ticks.length > 0) {
      let firstTick = this.ticks[0];
      return firstTick.x();
    } else {
      return x_start;
    }
  }

  if (this.ticks.length > 0) {
    this.original_stop_time = (this.ticks.length-1) * TIME_STEP;
  } else {
    this.original_stop_time = 0;
  }

  this.recenter = function() {
    let x_timeline_end = this.length();
    let x_timeline_start = this.start();
    let x_timeline_center = (x_timeline_end - x_timeline_start) / 2 + x_timeline_start;

    if (x_timeline_start > x_start) {
      let dx = Math.max(x_start - x_timeline_start, x_center - x_timeline_center);
      if (dx < -2 || dx > 2) { // past threshold then adjust
        timeline.move(dx,dx);
      }
    }
  }

}

//==============================================================================
// Events
//==============================================================================

SVG.on(document, 'DOMContentLoaded', function() {
  canvas = SVG('drawing');
  canvas.clear();
  let x = canvas.node.clientWidth;
  let y = canvas.node.clientHeight;
  timeline = new Timeline(canvas,0,x,x/2,y/2-50,[]);
});

$.getJSON("../rosbridge_properties.json", function(json) {
  // Setup ROS Subscribers

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

  var listenerRadTimeline = new ROSLIB.Topic({
    ros: ros,
    name: '/rad/timeline',
    'messageType': 'std_msgs/String'
  });

  let currentMode = -1;

  listenerRadSignal.subscribe(function(message) {
    if (message != undefined) {

      if (currentMode != message.mode) {
        currentMode = message.mode;
      }

      timeline.update(message.elapsed_time,message.mode != 0);
    }
  });

  listenerRadTimeline.subscribe(function(message) {
    if (message != undefined) {
      times = JSON.parse(message.data).timeline;
      currentMode = -1;

      // draw timeline
      canvas.clear();
      let x_window = canvas.node.clientWidth - 50;
      let y = canvas.node.clientHeight;
      timeline = new Timeline(canvas,50,x_window,50,y/2-50,times);

      // recenter timeline
      let x_timeline = timeline.length();
      if (x_timeline < x_window) {
        let dx = (x_window - x_timeline)/2;
        timeline.move(dx,dx);
      }
    }
  });
});
