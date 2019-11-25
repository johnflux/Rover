import React from 'react';
import './RosNodeHealth.css';

import { Paper, Table, TableBody, TableCell, TableHead, TableRow, Tooltip } from '@material-ui/core';
import { RosMonNode } from '../ROS_message_types';

type RosNodeHealthProps = {
  nodes: [RosMonNode]
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
        <Table size="small">
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
                <TableRow key={node.name}>
                  <TableCell>{node.name}</TableCell>
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
