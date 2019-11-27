import React from 'react';
import './WebsocketUrlInput.css';
import { Power, PowerOff } from '@material-ui/icons';
import { Input, InputAdornment, CircularProgress } from '@material-ui/core';

type WebsocketUrlInputProps = {
  //children?: React.ReactNode,
  hostname: string
  //lastHostname: string, /*< The last hostname that we tried to connect to */
  websocketStatus: "Disconnected" | "Connected",
  onSubmit: Function,
  lastMessageTimestamp: number,
}

type WebsocketUrlInputState = {
  lastHostname?: string
}

function timestampToTimeStr(unix_timestamp?: number) {
  if (!unix_timestamp)
    return "";

  var date = new Date(unix_timestamp * 1e3);
  return date.getHours().toString().padStart(2, '0') + ":" + date.getMinutes().toString().padStart(2, '0') + ":" + date.getSeconds().toString().padStart(2, '0');
}

class WebsocketUrlInput extends React.Component<WebsocketUrlInputProps, WebsocketUrlInputState> {
  constructor(props: WebsocketUrlInputProps) {
    super(props);
    this.state = {
      lastHostname: undefined
    };
  }

  handleChange(value?: string) {
    if (value === this.props.hostname)
      value = undefined;
    this.setState({lastHostname: value});
  }

  handleSubmit(value: string) {
    this.setState({lastHostname: undefined});
    this.props.onSubmit(value);
  }

  handleKeyDown = (e: { key: string; }) => {
    if (e.key === 'Enter') {
      this.handleSubmit(this.state.lastHostname || this.props.hostname);
    }
  }

  render() {
    const isConnected = this.props.websocketStatus === "Connected";
    const timestamp = timestampToTimeStr(this.props.lastMessageTimestamp);
    return (
      <div className="WebsocketUrlInput">
        <Input
          value={this.state.lastHostname || this.props.hostname}
          className={this.state.lastHostname !== undefined ? "Changed" : this.props.websocketStatus}
          onChange={e => this.handleChange(e.target.value)}
          onBlur={e => this.handleSubmit(e.target.value)}
          onKeyDown={this.handleKeyDown}
          endAdornment={
            <InputAdornment position="end">
              { isConnected ? null : <CircularProgress size="20px"/> }
              { isConnected ? <Power /> : <PowerOff /> }
            </InputAdornment>
          }
          />
          <span className="timestamp">
            { timestamp ? "[" + timestamp + "]" : null }
          </span>
      </div>
    );
  }
}
export default WebsocketUrlInput;
