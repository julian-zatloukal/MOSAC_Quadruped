// What the page and the server say to each other over the WebRTC data channel (Teleoperation/protocol.py describes both ends).

export type Command =
  | { cmd: 'stop' }
  // forward: fraction of the full stride, -1..1, negative = backwards. turn: fraction of the full turn, -1..1, positive = left.
  | { cmd: 'drive'; forward: number; turn: number }

export type RobotMode = 'stop' | 'walk' | 'back' | 'turn'

export interface RobotStatus {
  yaw_deg: number
  x_m?: number
  y_m?: number
  mode: RobotMode
  idle: boolean
  forward: number
  rate_deg_s: number
  max_rate_deg_s: number
}

// Pushed by the server every 100 ms.
export interface ServerState {
  connected: boolean
  age: number | null
  robot: RobotStatus | null
  video: { live: boolean; viewers: number }
}

export const CHANNEL_LABEL = 'teleop'

// A command that is not delivered within this long is dropped instead of retransmitted: by then the next one is already on its
// way, and the channel is ordered, so waiting for a stale command would hold the new ones up.
export const COMMAND_LIFETIME_MS = 200

// A held button is repeated this often; the robot stops by itself if the repeats stop.
export const HEARTBEAT_MS = 100
