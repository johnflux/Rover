import React from 'react';
import { Snackbar, IconButton } from '@material-ui/core';
import CloseIcon from '@material-ui/icons/Close';
import ErrorIcon from '@material-ui/icons/Error';
import './MySnackbar.css';

type MySnackbarProps = {
  errorMessages?: string[]
  onClose: () => void,
}

const MySnackbar: React.SFC<MySnackbarProps> = (props) =>
    <Snackbar
        className="snackbar-error"
        anchorOrigin={{
            vertical: 'top',
            horizontal: 'center',
        }}
        open={props.errorMessages !== undefined}
        autoHideDuration={6000}
        onClose={props.onClose}
        ContentProps={{
            'aria-describedby': 'message-id',
        }}
        message={
            <span id="message-id" className="snackbar-message">
                <ErrorIcon />
                <div className="snackbar-messagelist">
                    {props.errorMessages && props.errorMessages.map((msg,index) => <div key={index}>{msg}</div>)}
                </div>
            </span>
        }
        action={[
            <IconButton key="close" aria-label="close" color="inherit" onClick={props.onClose}>
                <CloseIcon/>
            </IconButton>
        ]}
    />;
export default MySnackbar;
