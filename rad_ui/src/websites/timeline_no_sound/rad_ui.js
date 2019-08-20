
// Global Variables

const NEGLECT_LOWER_BOUND = 15;

var timeline = null;
var playhead = null;

// global functions

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

  var timeStr = pad(minutes,2) + ':'
              + pad(seconds,2);
  return timeStr;
}

// Object Constructors

function Playhead(canvas, x, y) {

  this.triangleTop = canvas.polygon('0,0 20,0 10,20').fill('#000');
  this.triangleBottom = canvas.polygon('0,140 20,140 10,120').fill('#000');
  this.line = canvas.line(10,15,10,125).attr({
    stroke: '#000',
    'stroke-width': 2
  });

  this.dmove = function(x, y) {
    this.triangleTop.dmove(x,y);
    this.triangleBottom.dmove(x,y);
    this.line.dmove(x,y);
  };
  this.dmove(x,y);
}

function Tile(canvas, x, y, width, color, timeInfo) {
  this.state = 'okay';
  this.timeInfo = timeInfo;
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

  this.setColor = function(color) {
    this.rectangle.fill(color);
  }
}

function Timeline(canvas, x_start, x_end, x_playhead, y, times) {

  const TIME_STEP = 15;
  const X_STEP = 50;

  const TILE_Y_OFFSET = -50;
  const TICK_Y_OFFSET = 80;

  this.tiles = [];
  this.ticks = [];
  this.labels = [];

  this.time = 0;
  this.tileIndex = 0;
  this.stop_time = (times.length > 0) ? times[times.length-1].stop_time : 0;

  this._drawTiles = function(start_x, y) {

    for (let t of times) {
      let width = (t.stop_time - t.start_time) * X_STEP / TIME_STEP;
      let x = start_x + t.start_time * X_STEP / TIME_STEP;
      let color = (t.type == 'interaction') ? '#ff4444' : '#00C851';
      this.tiles.push(new Tile(canvas, x, y, width, color, t));
    }
  };
  this._drawTiles(x_playhead,y+TILE_Y_OFFSET);

  this._drawTicks = function(start_x, start_time, stop_time, y, major_ticks=true, minor_ticks=true) {
    let x = start_x;

    for (let t=start_time; t<stop_time+TIME_STEP; t+=TIME_STEP) {

      if (t % 60 == 0) {
        this.ticks.push(canvas.line(x,y,x,y+20).attr({
          stroke: '#000',
          'stroke-width': 2
        }));
      } else {
        this.ticks.push(canvas.line(x,y,x,y+10).attr({
          stroke: '#000',
          'stroke-width': 1
        }));
      }

      if (t % 60 == 0 && major_ticks) {
        this.labels.push(canvas.text(formatTime(t)).move(x-20,y+35));
      } else if (minor_ticks) {
        this.labels.push(canvas.text(formatTime(t)).move(x-20,y+35));
      }

      x += X_STEP;
    }
  };
  this._drawTicks(x_playhead,0,this.stop_time,y+TICK_Y_OFFSET);

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

        // update tick marks
        lastTick = this.ticks[this.ticks.length-1];
        this._drawTicks(lastTick.x() + X_STEP,this.ticks.length*TIME_STEP,this.stop_time+dt,y+TICK_Y_OFFSET);

        this.stop_time += dt;
      } else if (activeTile.timeInfo.type == 'interaction' && !interacting) {
        let qt = time - activeTile.timeInfo.stop_time - 0.01;
        let qx = qt * X_STEP / TIME_STEP;

        // adjust tiles
        activeTile.extend(qx,qt);
        for (let i=this.tileIndex+1; i<this.tiles.length; i++) {
          this.tiles[i].timeAdjust(qx,qt);
        }

        // update tick marks
        for (let i=this.ticks.length-1; i>0; i--) {
          if ((i-1)*TIME_STEP >= this.stop_time) {
            this.ticks.pop().clear();
            this.labels.pop().clear();
          }
        }
      } else if (activeTile.timeInfo.type == 'neglect') {
        if (activeTile.state != 'warning' && activeTile.timeInfo.stop_time - (this.time+dt) < 15) {
          activeTile.state = 'warning';
          activeTile.setColor('#ffbb33');
        }
      }
    }

    // shift everything over
    for (let t of this.tiles) {
      t.dmove(dx,0);
    }
    for (let t of this.ticks) {
      t.dmove(dx,0);
    }
    for (let l of this.labels) {
      l.dmove(dx,0);
    }

    // update timing and active tile
    this.time = time;
    while (this.tileIndex < this.tiles.length && this.time > this.tiles[this.tileIndex].timeInfo.stop_time) {
      this.tiles[this.tileIndex].setColor('#555');
      this.tileIndex++;
    }
  }

}

// Events

SVG.on(document, 'DOMContentLoaded', function() {
  canvas = SVG('drawing');
  canvas.clear();
  let x = canvas.node.clientWidth;
  let y = canvas.node.clientHeight;

  timeline = new Timeline(canvas,0,x,x/3,y/2-50,[]);
  playhead = new Playhead(canvas,x/3-10,y/2-120);

  /*
  [
    {
      start_time: 0,
      stop_time: 60,
      type: 'neglect'
    },
    {
      start_time: 61,
      stop_time: 78,
      type: 'interaction'
    },
    {
      start_time: 90,
      stop_time: 120,
      type: 'neglect'
    }
  ]
  */

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
      times = JSON.parse(message.data);

      currentMode = -1;

      canvas.clear();
      let x = canvas.node.clientWidth;
      let y = canvas.node.clientHeight;

      timeline = new Timeline(canvas,0,x,x/3,y/2-50,times);
      playhead = new Playhead(canvas,x/3-10,y/2-120);
    }
  });
});
