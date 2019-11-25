export type RosMonNode = {
  "user_load": number,
  "name": string,
  "restart_count": number,
  "state": number, /* 0 = idle, 1 = running, 2 = crashed, 3 = waiting */
  "memory": number, /* bytes */
  "ns": string, /* Not sure what this means */
  "system_load": number,
}

export type RosMon = {
  header: {
    stamp: {
      secs: number
    }
  },
  nodes: [RosMonNode]
}

export type SensorMsgsJoy = {
  "header"?: {
    "stamp": {
      "secs": number,
      "nsecs": number
    },
    "frame_id": string,
    "seq": number
  },
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
