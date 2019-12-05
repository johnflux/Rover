import React from 'react';
import { IconButton, Toolbar, AppBar, Typography } from '@material-ui/core';
import PowerSettingsNewIcon from '@material-ui/icons/PowerSettingsNew';
import SyncIcon from '@material-ui/icons/Sync';
import WebsocketUrlInput from '../WebsocketUrlInput/WebsocketUrlInput';


type MyAppBarProps = {
    onPowerOff: () => void,
    onPowerReboot: () => void,
    hostname: string,
    websocketStatus: "Disconnected" | "Connected",
    onHostnameSubmit: (newHostname: string) => void,
    lastMessageTimestamp: number,
}

const MyAppBar: React.SFC<MyAppBarProps> = (props) =>
    <AppBar position="static">
        <Toolbar>
            <IconButton key="power" aria-label="power-off" disabled={props.websocketStatus !== "Connected"}
                color="secondary" onClick={props.onPowerOff}>
                <PowerSettingsNewIcon fontSize="large"/>
            </IconButton>
            <IconButton key="reboot" aria-label="reboot" disabled={props.websocketStatus !== "Connected"}
                color="secondary" onClick={props.onPowerReboot}>
                <SyncIcon fontSize="large"/>
            </IconButton>
            <WebsocketUrlInput
                hostname={props.hostname}
                websocketStatus={props.websocketStatus}
                onSubmit={props.onHostnameSubmit}
                lastMessageTimestamp={props.lastMessageTimestamp}
                />
            <Typography variant="h6">[33:33:33]</Typography>
        </Toolbar>
    </AppBar>;
export default MyAppBar;
