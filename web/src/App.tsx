import React from 'react';
import './App.css';
import Joystick from './Joystick';
import ROSLIB from 'roslib';
import WebsocketUrlInput from './WebsocketUrlInput/WebsocketUrlInput';
import RosNodeHealth from './RosNodeHealth/RosNodeHealth';
import { Paper } from '@material-ui/core';


type AppState = {
  [rosmon: string]: any,
  hostname: string,
  websocketStatus: "Disconnected" | "Connected",
}

class App extends React.Component<{},AppState> {
  ros: any;
  constructor(props: {}) {
    super(props);
    this.state = {
      rosmon: {},
      hostname: localStorage.getItem('websocket_url') || ('ws://' + window.location.hostname + ':9090'),
      websocketStatus: "Disconnected",
    }

    this.ros = new ROSLIB.Ros({
      url: this.state.hostname
    });

    this.ros.on('connection', () => {
      console.log('Connected to websocket server.');
      this.setState({websocketStatus: "Connected"});
    });

    this.ros.on('error', (error: string) => {
      console.log('Error connecting to websocket server: ', error);
    });

    this.ros.on('close', () => {
      console.log('Connection to websocket server closed.');
      this.setState({websocketStatus: "Disconnected"});
      this.ros.connect(this.state.hostname);
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
    }).subscribe((message) => {
      //console.log('Received message on /rosmon/state:', message);
      this.setState({rosmon: message});
    });

  }

  connectToWebsocket() {
    this.ros.close();
    this.setState({websocketStatus: "Disconnected"});
  }

  onHostnameSubmit = (newHostname: string) => {
    localStorage.setItem('websocket_url', newHostname);
    this.setState({ hostname: newHostname}, () => this.connectToWebsocket());
  }

  render() {
    return (
      <div className="App">
        { this.state.rosmon && this.state.rosmon.nodes &&
          <RosNodeHealth nodes={this.state.rosmon.nodes} />
        }
        <WebsocketUrlInput
            hostname={this.state.hostname}
            websocketStatus={this.state.websocketStatus}
            onSubmit={this.onHostnameSubmit}
            lastMessageTimestamp={this.state.rosmon.header ? this.state.rosmon.header.stamp.secs : 0}
            />
        <Joystick />
      </div>
    );
  }
}

export default App;
