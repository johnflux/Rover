import "joypad.js";
import { SensorMsgsJoy } from "./ROS_message_types";

function deadzone(x: number) {
    if ( x > -0.1 && x < 0.1)
        return 0;
    return x;
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
            buttons: this.joypad.instances[0].buttons.map((x: any) => x.value as number),
            axes: this.joypad.instances[0].axes.map((v:number) => deadzone(-v)),
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
