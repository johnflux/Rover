import React from 'react';
import './RosOutLog.css';

import { Paper, Table, TableBody, TableCell, TableRow } from '@material-ui/core';
import { RosOut } from '../ROS_message_types';

type RosOutLogProps = {
  rosouts: RosOut[],
  websocketStatus: "Disconnected" | "Connected",
}

const RosOutLog: React.SFC<RosOutLogProps> = (props) => {
  return (
    <div className="RosOutLog">
      <Paper>
        <Table size="small" className={props.websocketStatus !== 'Connected' ? 'notconnected' : ''}>
          <TableBody>
              { props.rosouts.map((rosout,index) =>
                <TableRow key={index} className={'rosout_level_' + rosout.level}>
                  <TableCell className="level_mark">▋</TableCell>
                  <TableCell>{rosout.name}</TableCell>
                  <TableCell style={{maxWidth: '20vw'}}>{rosout.msg}</TableCell>
                </TableRow>
              ) }
          </TableBody>
        </Table>
      </Paper>
    </div>
  );
}
export default RosOutLog;
