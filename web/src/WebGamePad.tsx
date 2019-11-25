import "joypad.js";
import { SensorMsgsJoy } from "./ROS_message_types";

const sensorMsgsJoyHeader = {
    "stamp": {
        "secs": 0,
        "nsecs": 0
    },
    "frame_id": "",
    "seq": 0
}

class WebGamePad {
    joypad: any;
    lastJoystickMessageJson: string = "";
    timeoutId?: any;
    constructor() {
        this.joypad = (window as any).joypad as any;
        this.joypad.on('connect', (e: any) => {
            const { id } = e.gamepad;

            console.log(`${id} connected!`);
        });
        this.joypad.on('button_press', () => { this.sendJoystickMessage(); this.resetTimeout()});
        this.joypad.on('axis_move', () => { this.sendJoystickMessage(); this.resetTimeout()});
    }
    resetTimeout() {
        if (this.timeoutId !== undefined)
            clearTimeout(this.timeoutId);
        this.timeoutId = setTimeout(this.sendJoystickMessage.bind(this), 100);
    }

    sendJoystickMessage() {
        const msg: SensorMsgsJoy = {
            header: sensorMsgsJoyHeader,
            buttons: this.joypad.instances[0].buttons.map((x: any) => x.value as number),
            axes: this.joypad.instances[0].axes.map((v:number) => -v),
        };
        const msgJson = JSON.stringify(msg);
        if (this.lastJoystickMessageJson === msgJson)
            return;
        this.lastJoystickMessageJson = msgJson;
        this.onJoystickMessage(msg);
    }

    onJoystickMessage = (msg: SensorMsgsJoy) => {}

}

export default WebGamePad;
