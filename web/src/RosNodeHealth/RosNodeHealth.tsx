import React from 'react';
import './RosNodeHealth.css';

import { Paper, Table, TableBody, TableCell, TableHead, TableRow, Tooltip } from '@material-ui/core';

type RosNodeHealthProps = {
  nodes: [
    {
      "user_load": number,
      "name": string,
      "restart_count": number,
      "state": number, /* Not sure what this means */
      "memory": number, /* bytes */
      "ns": string, /* Not sure what this means */
      "system_load": number,
    }
  ],
}

class RosNodeHealth extends React.Component<RosNodeHealthProps> {
  render() {
    return (
      <div className="RosNodeHealth">
        <Paper>
          <Table size="small">
            <TableHead>
              <TableRow>
                <TableCell>Name</TableCell>
                <TableCell><Tooltip title="Number of Restarts"><span>Restart</span></Tooltip></TableCell>
                <TableCell>State</TableCell>
                <TableCell align="right">Mem (KB)</TableCell>
                <TableCell align="right">User</TableCell>
                <TableCell align="right">Sys</TableCell>
              </TableRow>
            </TableHead>
            <TableBody>
                { this.props.nodes.map(node =>
                  <TableRow key={node.name}>
                    <TableCell>{node.name}</TableCell>
                    <TableCell>{node.restart_count}</TableCell>
                    <TableCell>{node.state}</TableCell>
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
}
export default RosNodeHealth;
