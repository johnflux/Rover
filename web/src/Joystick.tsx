import React from 'react';
import './App.css';

const Joystick: React.FC = () => {
  return (
    <div className="Joystick">
      <svg width="400" height="400">
        <image xlinkHref="controller.jpg" height="400" width="400" />
      </svg>
    </div>
  );
}

export default Joystick;
