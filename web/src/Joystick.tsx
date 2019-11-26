import React from 'react';
import './App.css';
import { SensorMsgsJoy } from './ROS_message_types';

type JoystickProps = {
  sensor_msgs_joy?: SensorMsgsJoy
}

const buttonPositionsCenter = [
  [283, 135],
  [305, 114],
  [261, 113],
  [283, 91],
  [121, 57],
  [277, 58],
  [175, 112],
  [223, 112],
  [200, 80],
  [117, 112],
  [242, 160],
];

const axesPositionsCenter = [
  [117, 112, 15, 0, 1],
  [99, 54, 15, -1, 2],
  [242, 160, 15, 3, 4],
  [300, 54, 15, -1, 5],
  [157, 165, 15, 6, 7],
];

const Joystick: React.SFC<JoystickProps> = (props) => {
  const sensor_msgs_joy = props.sensor_msgs_joy;
  return (
    <div className="Joystick">
      <svg width="400" height="348">
        <image xlinkHref="controller.png" x="0" y="0" height="348" width="400" />
        { sensor_msgs_joy && buttonPositionsCenter.map((positions, index) =>
          sensor_msgs_joy.buttons[index] &&
            <circle key={index} cx={positions[0]} cy={positions[1]} r={12} fill="#ad009e" style={{opacity: 0.8}}/>
          )
        }
        { sensor_msgs_joy && axesPositionsCenter.map((positions, index) => {
          const scale = positions[2];
          const x = positions[3] === -1 ? 0 : sensor_msgs_joy.axes[positions[3]];
          const y = sensor_msgs_joy.axes[positions[4]];
          if ( (positions[3] === -1 && y === 1) || (positions[3] !== -1 && x === 0 && y === 0))
            return null;
          return <circle key={index} cx={positions[0] - (x * scale)} cy={positions[1] - y * scale} r={8} fill="#00e09e" style={{opacity: 0.8}}/>
          })
        }
      </svg>
    </div>
  );
}

export default Joystick;
