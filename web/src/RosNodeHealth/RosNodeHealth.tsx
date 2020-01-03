import React from 'react';
import './RosNodeHealth.css';

import { Paper, Table, TableBody, TableCell, TableHead, TableRow, Tooltip, IconButton } from '@material-ui/core';
import StopIcon from '@material-ui/icons/Stop';
import PlayArrowIcon from '@material-ui/icons/PlayArrow';
import { RosMonNode, RosmonActionEnum } from '../ROS_message_types';

type RosNodeHealthProps = {
  nodes: RosMonNode[],
  websocketStatus: "Disconnected" | "Connected",
  onStartStopRosNode: (nodeName: string, namespace: string, action: RosmonActionEnum) => (void),
}

const stateToStr = [
  "Exited", "R", "Crashed", "Waiting for restart"
];

const RosNodeHealth: React.SFC<RosNodeHealthProps> = (props) => {
  let hasStateNotRunning = false;
  let hasRestarts = false;
  props.nodes.forEach(node => {
    if (node.restart_count !== 0)
      hasRestarts = true;
    if (node.state !== 1)
      hasStateNotRunning = true;
  });

  return (
    <div className="RosNodeHealth">
      <Paper>
        <Table size="small" className={props.websocketStatus !== 'Connected' ? 'notconnected' : ''}>
          <TableHead>
            <TableRow>
              <TableCell>Name</TableCell>
              {hasRestarts && <TableCell><Tooltip title="Number of Restarts"><span>Restart</span></Tooltip></TableCell>}
              {hasStateNotRunning && <TableCell><Tooltip title="R = Running"><span>State</span></Tooltip></TableCell>}
              <TableCell align="right">Mem (KB)</TableCell>
              <TableCell align="right">User %</TableCell>
              <TableCell align="right">Sys %</TableCell>
            </TableRow>
          </TableHead>
          <TableBody>
              { props.nodes.map(node =>
                <TableRow key={node.name} className={"tablerow " + (node.state !== 1 ? 'crashed' : '')}>
                  <TableCell>
                    { node.state === 1 ?
                      <IconButton className="processIconButton" key="processRebootIcon" aria-label="process-reboot" disabled={props.websocketStatus !== "Connected"} color="secondary" onClick={() => props.onStartStopRosNode(node.name, node.ns, RosmonActionEnum.STOP)} size="small">
                        <StopIcon fontSize="small"/>
                      </IconButton>
                      :
                      <IconButton className="processIconButton" key="processRebootIcon" aria-label="process-reboot" disabled={props.websocketStatus !== "Connected"} color="secondary" onClick={() => props.onStartStopRosNode(node.name, node.ns, RosmonActionEnum.START)} size="small">
                        <PlayArrowIcon fontSize="small" htmlColor="lightgreen"/>
                      </IconButton>
                    }
                    {node.ns} {node.name}
                  </TableCell>
                  {hasRestarts && <TableCell>{node.restart_count}</TableCell>}
                  {hasStateNotRunning && <TableCell>{stateToStr[node.state]}</TableCell>}
                  <TableCell align="right">{node.memory/1024}</TableCell>
                  <TableCell align="right">{(node.user_load*100).toFixed(1)}</TableCell>
                  <TableCell align="right">{(node.system_load*100).toFixed(1)}</TableCell>
                </TableRow>
              ) }
          </TableBody>
        </Table>
      </Paper>
    </div>
  );
}
export default RosNodeHealth;
