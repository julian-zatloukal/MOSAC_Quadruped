import { CHANNEL_LABEL, COMMAND_LIFETIME_MS, type Command, type ServerState } from './protocol.ts'

// 'connecting' until the server has answered for the first time, 'down' after it was lost (until it is back).
export type Link = 'connecting' | 'up' | 'down'

export interface ConnectionEvents {
  onLink: (link: Link) => void
  onState: (state: ServerState | null) => void
  onStream: (stream: MediaStream | null) => void
}

const RETRY_MS = [500, 1000, 2000, 4000]
const GATHER_TIMEOUT_MS = 1000 // how long to wait for the browser's own candidates before sending the offer
const SILENCE_MS = 1500 // the server pushes its state ten times a second: this long without one and the link is lost

// WebRTC with H.264 in the browser; the server only offers H.264 (it passes on what the robot's encoder made).
export function canReceiveVideo(): boolean {
  if (typeof RTCPeerConnection === 'undefined') return false
  const codecs = RTCRtpReceiver.getCapabilities?.('video')?.codecs ?? []
  return codecs.some((codec) => codec.mimeType.toLowerCase() === 'video/h264')
}

function gathered(pc: RTCPeerConnection): Promise<void> {
  if (pc.iceGatheringState === 'complete') return Promise.resolve()
  return new Promise((resolve) => {
    const done = () => {
      clearTimeout(timer)
      pc.removeEventListener('icegatheringstatechange', check)
      resolve()
    }
    const check = () => {
      if (pc.iceGatheringState === 'complete') done()
    }
    const timer = setTimeout(done, GATHER_TIMEOUT_MS)
    pc.addEventListener('icegatheringstatechange', check)
  })
}

// The page's one connection to the server: the camera as a video track, and a data channel for the commands (out) and the
// robot's state (in). Reconnects by itself while started.
export class TeleopConnection {
  private events: ConnectionEvents
  private pc: RTCPeerConnection | null = null
  private channel: RTCDataChannel | null = null
  private running = false
  private link: Link = 'connecting'
  private retries = 0
  private retryTimer: number | undefined
  private watchdog: number | undefined

  constructor(events: ConnectionEvents) {
    this.events = events
  }

  start() {
    if (this.running) return
    this.running = true
    void this.connect()
  }

  stop() {
    this.running = false
    clearTimeout(this.retryTimer)
    this.close()
    this.setLink('connecting')
    this.events.onState(null)
    this.events.onStream(null)
  }

  // False when the command could not be sent (no connection); the robot then stops by itself.
  send = (command: Command): boolean => {
    if (this.channel?.readyState !== 'open') return false
    this.channel.send(JSON.stringify(command))
    return true
  }

  // The WebRTC statistics of the connection, or null while there is none.
  getStats = (): Promise<RTCStatsReport> | null => this.pc?.getStats() ?? null

  private setLink(link: Link) {
    if (link === this.link) return
    this.link = link
    this.events.onLink(link)
  }

  private close() {
    clearTimeout(this.watchdog)
    const pc = this.pc
    this.pc = this.channel = null
    if (pc) {
      pc.ontrack = pc.onconnectionstatechange = null
      pc.close()
    }
  }

  private async connect() {
    const pc = new RTCPeerConnection({ iceServers: [] }) // no STUN: the server is on this computer or its network
    this.pc = pc
    try {
      pc.addTransceiver('video', { direction: 'recvonly' })
      const channel = pc.createDataChannel(CHANNEL_LABEL, { ordered: true, maxPacketLifeTime: COMMAND_LIFETIME_MS })
      this.channel = channel
      pc.ontrack = (event) => this.events.onStream(new MediaStream([event.track]))
      pc.onconnectionstatechange = () => {
        if (pc.connectionState === 'failed' || pc.connectionState === 'closed') this.lost(pc)
      }
      channel.onclose = () => this.lost(pc)
      channel.onmessage = (event: MessageEvent<string>) => this.heard(pc, event.data)

      await pc.setLocalDescription(await pc.createOffer())
      await gathered(pc)
      if (this.pc !== pc) return // stopped, or lost, meanwhile
      const response = await fetch('/api/offer', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ sdp: pc.localDescription?.sdp, type: pc.localDescription?.type }),
      })
      if (!response.ok) throw new Error(`the server answered ${response.status}`)
      const answer = (await response.json()) as RTCSessionDescriptionInit
      if (this.pc !== pc) return
      await pc.setRemoteDescription(answer)
    } catch {
      this.lost(pc)
    }
  }

  private heard(pc: RTCPeerConnection, data: string) {
    if (this.pc !== pc) return
    this.retries = 0
    this.setLink('up')
    clearTimeout(this.watchdog)
    this.watchdog = setTimeout(() => this.lost(pc), SILENCE_MS)
    this.events.onState(JSON.parse(data) as ServerState)
  }

  // pc is the connection that failed; a late report about one that was replaced already is ignored.
  private lost(pc: RTCPeerConnection) {
    if (this.pc !== pc) return
    this.close()
    this.setLink('down')
    this.events.onState(null)
    this.events.onStream(null)
    if (this.running) {
      this.retryTimer = setTimeout(() => void this.connect(), RETRY_MS[Math.min(this.retries++, RETRY_MS.length - 1)])
    }
  }
}
