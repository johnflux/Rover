/* See https://github.com/xqms/rosmon/blob/master/rosmon_msgs/msg/NodeState.msg */
export type RosMonNode = {
  "user_load": number,
  "name": string, /* Ros node namespace */
  "restart_count": number,
  "state": number, /* 0 = idle, 1 = running, 2 = crashed, 3 = waiting */
  "memory": number, /* bytes */
  "ns": string, /* Ros node namespace */
  "system_load": number,
}

type Header = {
  "stamp": {
    "secs": number,
    "nsecs": number
  },
  "frame_id": string,
  "seq": number
}

/** As defined by rosmon_msgs/srv/StartStop.srv
   https://github.com/xqms/rosmon/blob/master/rosmon_msgs/srv/StartStop.srv
   */
export enum RosmonActionEnum {
  START = 1,
  STOP = 2,
  RESTART = 3
}

export type RosMon = {
  header: Header,
  nodes: RosMonNode[]
}

export type RosOut = {
  header: Header,
  level: number,    // 1=debug, 2=info, 4=warn, 8=error, 16=fatal
  name: string,     // name of the node
  msg: string,
  file: string,     // file the message came from
  function: string, // function the message came from
  line: number,     // line the message came from
  topics: string[], // topic names that the node publishes
}

export type SensorMsgsJoy = {
  "header"?: Header,
  "buttons": [
    number, /* A */
    number, /* B */
    number, /* X */
    number, /* Y */
    number, /* LB */
    number, /* RB */
    number, /* Back button */
    number, /* Start button */
    number, /* Big Silver button */
    number, /* Left axes push */
    number, /* Right axes push */
  ],
  "axes": [
    number,
    number,
    number,
    number,
    number,
    number,
    number,
    number
  ]
}
