import React from 'react';
import './App.css';
import Joystick from './Joystick';
import './libs/eventemitter2.min.js';
import ROSLIB from './libs/roslib.min.js';

type AppState = {
  rosmon: any
}

class App extends React.Component<{},AppState> {
  ros: any;
  constructor(props: {}) {
    super(props);
    this.state = {
      rosmon: {}
    }

    this.ros = new ROSLIB.Ros({
      url: 'ws://' + window.location.hostname + ':9090'
    });

    this.ros.on('connection', function () {
      console.log('Connected to websocket server.');
    });

    this.ros.on('error', function (error) {
      console.log('Error connecting to websocket server: ', error);
    });

    this.ros.on('close', function () {
      console.log('Connection to websocket server closed.');
    });

    /*new ROSLIB.Topic({
      ros: this.ros,
      name: '/cmd_vel',
      messageType: 'geometry_msgs/Twist'
    }).subscribe(function (message) {
      console.log('Received message on /cmd_vel:', message);
    });*/

    new ROSLIB.Topic({
      ros: this.ros,
      name: '/joy',
      messageType: 'sensor_msgs/Joy'
    }).subscribe(function (message) {
      console.log('Received message on /joy:', message);
    });

    new ROSLIB.Topic({
      ros: this.ros,
      name: '/rosmon/state',
      messageType: 'rosmon_msgs/State'
    }).subscribe(function (message) {
      console.log('Received message on /rosmon/state:', message);
    });
  }

  render() {
    return (
      <div className="App">
        <header className="App-header">
          <p>
            Rover - Flux Programming Ltd
          </p>
        </header>
        Hellosadf
        <Joystick />
      </div>
    );
  }
}

export default App;
