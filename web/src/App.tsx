import React from 'react';
import './App.css';
import Joystick from './Joystick';
import ROSLIB from 'roslib';
import RosNodeHealth from './RosNodeHealth/RosNodeHealth';
import { RosMon, RosOut, SensorMsgsJoy } from './ROS_message_types';
import WebGamePad from './WebGamePad';
import RosOutLog from './RosOutLog/RosOutLog';
import { Drawer, Paper } from '@material-ui/core';
import MySnackbar from './MySnackbar/MySnackbar';
import OnScreenJoystick from './OnScreenJoystick/OnScreenJoystick';
import MyAppBar from './MyAppBar/MyAppBar';

type AppState = {
  rosmon?: RosMon,
  rosouts: RosOut[],
  sensor_msgs_joy?: SensorMsgsJoy,
  hostname: string,
  websocketStatus: "Disconnected" | "Connected",
  reconnect_count: number,
  errorMessages?: string[],
}

class App extends React.Component<{},AppState> {
  ros: any;
  gamepad: WebGamePad;
  sensorMsgTopic: ROSLIB.Topic;
  service_poweroff: ROSLIB.Service;
  service_reboot: ROSLIB.Service;
  constructor(props: {}) {
    super(props);
    this.state = {
      rosmon: undefined,
      rosouts: [],
      sensor_msgs_joy: undefined,
      hostname: localStorage.getItem('websocket_url') || window.location.hostname,
      websocketStatus: "Disconnected",
      reconnect_count: 0,
      errorMessages: undefined,
    }
    this.gamepad = new WebGamePad();
    this.gamepad.onJoystickMessage = (msg: SensorMsgsJoy) => this.handleJoystickMessage(msg);

    this.ros = new ROSLIB.Ros({
      url: "ws://" + this.state.hostname + ":9090"
    });

    this.ros.on('connection', () => {
      console.log('Connected to websocket server.');
      this.setState({websocketStatus: "Connected", rosouts: [], rosmon: undefined, sensor_msgs_joy: undefined, reconnect_count: this.state.reconnect_count+1});
    });

    this.ros.on('error', (error: string) => {
      console.log('Error connecting to websocket server: ', error);
      this.showErrorMessage("Error connecting to websocket server " + (error as any).target.url);
    });

    this.ros.on('close', () => {
      console.log('Connection to websocket server closed.');
      this.setState({websocketStatus: "Disconnected"});
      this.ros.connect("ws://" + this.state.hostname + ":9090");
    });

    /*new ROSLIB.Topic({
      ros: this.ros,
      name: '/cmd_vel',
      messageType: 'geometry_msgs/Twist'
    }).subscribe(function (message) {
      console.log('Received message on /cmd_vel:', message);
    });*/

    this.sensorMsgTopic = new ROSLIB.Topic({
      ros: this.ros,
      name: '/joy',
      messageType: 'sensor_msgs/Joy'
    });
    this.sensorMsgTopic.subscribe((message) => {
      //console.log('Received message on /joy:', message);
      this.setState({sensor_msgs_joy: message as SensorMsgsJoy});
    });

    new ROSLIB.Topic({
      ros: this.ros,
      name: '/rosmon/state',
      messageType: 'rosmon_msgs/State'
    }).subscribe((message) => {
      //console.log('Received message on /rosmon/state:', message);
      this.setState({rosmon: message as RosMon});
    });

    new ROSLIB.Topic({
      ros: this.ros,
      name: '/rosout',
      messageType: 'rosgraph_msgs/Log'
    }).subscribe((message) => {
      //console.log('Received message on /rosout:', message);
      var rosouts = this.state.rosouts;
      const max_rosouts = 6;
      if (rosouts.length >= max_rosouts) {
        rosouts = rosouts.slice(rosouts.length - max_rosouts + 1);
      }
      this.setState({rosouts: [...rosouts, (message as RosOut)]});
    });

    // Note: You can use 'rosservice list' to see the services running
    //       and rosservice type SERVICENAME' to see the serviceType for it
    this.service_poweroff = new ROSLIB.Service({
      ros : this.ros,
      name : '/power_off/power_off',
      serviceType : 'rover/PowerOff'
    });
    this.service_reboot = new ROSLIB.Service({
      ros : this.ros,
      name : '/power_off/reboot',
      serviceType : 'rover/PowerOff'
    });

  }

  handleJoystickMessage(msg: SensorMsgsJoy) {
    if (this.state.websocketStatus === "Connected") {
      // We don't need to manually set the state, because
      // we also subscribe to this topic
      this.sensorMsgTopic.publish(new ROSLIB.Message(msg));
    } else {
      // We are not connected to the websocket, but it's nice
      // to see the gui respond anyway
      this.setState({sensor_msgs_joy: msg});
    }
  }

  connectToWebsocket() {
    this.ros.close();
    this.setState({websocketStatus: "Disconnected"});
  }

  onHostnameSubmit = (newHostname: string) => {
    localStorage.setItem('websocket_url', newHostname);
    this.setState({ hostname: newHostname}, () => this.connectToWebsocket());
  }

  showErrorMessage(message: string) {
    //if (this.state.errorMessages === undefined)
      this.setState({errorMessages: [message]});
    //else
    //  this.setState({errorMessages: [...this.state.errorMessages, message]});
  }

  clearErrorMessages = () => {
    this.setState({errorMessages: undefined});
  }

  handlePowerOffClicked = () => {
    console.log("PowerOff Clicked");
    this.service_poweroff.callService(new ROSLIB.ServiceRequest(), (response) =>
    {
      console.log("PowerOff response from rover was", response);
      if (response.process_exit_code !== 0)
        this.showErrorMessage("Failed to power off - response: " + response.process_exit_code);
    });
    this.setState({websocketStatus: "Disconnected"});
    setTimeout(() => this.ros.close(), 3000); // Force close and auto-attempt reconnection
  }
  handleRebootClicked = () => {
    console.log("Reboot Clicked");
    this.service_reboot.callService(new ROSLIB.ServiceRequest(), (response) =>
    {
      console.log("Reboot response from rover was", response);
      if (response.process_exit_code !== 0)
        this.showErrorMessage("Failed to reboot - response: " + response.process_exit_code);
    });
    this.setState({websocketStatus: "Disconnected"});
    setTimeout(() => this.ros.close(), 3000); // Force close and auto-attempt reconnection
  }

  handleOnScreenJoystickMove = (x: number, y: number) => {
    let msg: SensorMsgsJoy = {
      buttons: [0,0,0,0,0,0,0,0,0,0,0],
      axes: [0,0,0,x,y,0,0,0]
    }

    if (this.state.websocketStatus === "Connected") {
      // We don't need to manually set the state, because
      // we also subscribe to this topic
      this.sensorMsgTopic.publish(new ROSLIB.Message(msg));
    } else {
      // We are not connected to the websocket, but it's nice
      // to see the gui respond anyway
      this.setState({sensor_msgs_joy: msg});
    }
  }

  render() {
    return (
      <div className="App" style={{background: "url(http://" + this.state.hostname + ":8080/stream?topic=/cv_camera/image_raw&type=ros_compressed&"+this.state.reconnect_count+") no-repeat center center fixed"}}>
        <MyAppBar
          onPowerOff={this.handlePowerOffClicked}
          onPowerReboot={this.handleRebootClicked}
          hostname={this.state.hostname}
          websocketStatus={this.state.websocketStatus}
          onHostnameSubmit={this.onHostnameSubmit}
          lastMessageTimestamp={this.state.rosmon ? this.state.rosmon.header.stamp.secs : 0}
          />
        <Drawer
          className="rightDrawer"
          variant="permanent"
          anchor="right"
          open>
          <Paper>
            <Joystick sensor_msgs_joy={this.state.sensor_msgs_joy}/>
          </Paper>
          { this.state.rosmon &&
            <RosNodeHealth nodes={this.state.rosmon.nodes} websocketStatus={this.state.websocketStatus}/>
          }
          { this.state.rosouts &&
            <RosOutLog rosouts={this.state.rosouts} websocketStatus={this.state.websocketStatus}/>
          }
        </Drawer>
        <OnScreenJoystick
          onMove={this.handleOnScreenJoystickMove}
          disabled={this.state.websocketStatus !== 'Connected'}
        />
        <MySnackbar
          errorMessages={this.state.errorMessages}
          onClose={this.clearErrorMessages}
          />
      </div>
    );
  }
}

export default App;
