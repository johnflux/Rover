import React from 'react';
import ReactNipple from 'react-nipple';

// optional: include the stylesheet somewhere in your app
import 'react-nipple/lib/styles.css';
import './OnScreenJoystick.css';

type OnScreenJoystickProps = {
    onMove: (x:number,y:number) => void,
    disabled: boolean
}

const OnScreenJoystick: React.SFC<OnScreenJoystickProps> = (props) =>
    <ReactNipple
        // supports all nipplejs options
        // see https://github.com/yoannmoinet/nipplejs#options
        options={{ mode: 'static', position: { top: '50%', left: '50%' }, color: 'green'}}
        // any unknown props will be passed to the container element, e.g. 'title', 'style' etc
        style={{
            width: 150,
            height: 150
            // if you pass position: 'relative', you don't need to import the stylesheet
        }}
        // all events supported by nipplejs are available as callbacks
        // see https://github.com/yoannmoinet/nipplejs#start
        onMove={(evt:any, data:any) =>
            props.onMove(
                2*(-data.position.x + evt.target.nipples[0].position.x)/evt.target.nipples[0].options.size,
                2*(-data.position.y + evt.target.nipples[0].position.y)/evt.target.nipples[0].options.size)}
        onEnd={() => { props.onMove(0,0)}}
    />
    ;
export default OnScreenJoystick;
