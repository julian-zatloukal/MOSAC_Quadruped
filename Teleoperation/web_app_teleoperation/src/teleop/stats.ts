// Link quality, from the connection's WebRTC statistics. Rates need two readings, so they are null on the first one.

export interface LinkStats {
  rttMs: number | null // round trip to the server
  bitrateKbps: number | null // video received
  lossPercent: number | null // video packets lost, of those sent
  jitterMs: number | null
  fps: number | null
  width: number | null
  height: number | null
  delayMs: number | null // in the browser, from the packet arriving to the picture decoded: jitter buffer + decode
  dropped: number | null // frames the browser gave up on, in total
  freezes: number | null // times the picture stopped for long enough to notice, in total
  nacks: number | null // retransmissions asked for, in total
  route: string | null
}

export const NO_STATS: LinkStats = {
  rttMs: null, bitrateKbps: null, lossPercent: null, jitterMs: null, fps: null, width: null, height: null, delayMs: null,
  dropped: null, freezes: null, nacks: null, route: null,
}

// The counters of one reading, to take the difference to the next.
export interface Sample {
  time: number
  bytes: number
  received: number
  lost: number
  frames: number
  decodeTime: number
  bufferDelay: number
  bufferCount: number
}

type Stat = Record<string, unknown>

const num = (stat: Stat | undefined, key: string): number | null => {
  const value = stat?.[key]
  return typeof value === 'number' ? value : null
}

const scale = (value: number | null, factor: number) => (value === null ? null : value * factor)

const ROUTES: Record<string, string> = { host: 'Direct', srflx: 'Direct', prflx: 'Direct', relay: 'Relayed' }

function route(local: Stat | undefined): string | null {
  if (!local) return null
  const parts = [ROUTES[String(local.candidateType)] ?? null, typeof local.protocol === 'string' ? local.protocol.toUpperCase() : null]
  if (typeof local.networkType === 'string' && local.networkType !== 'unknown') parts.push(local.networkType)
  return parts.filter(Boolean).join(' · ') || null
}

export function readStats(report: RTCStatsReport, previous: Sample | null): { stats: LinkStats; sample: Sample | null } {
  let video: Stat | undefined
  let transport: Stat | undefined
  const pairs: Stat[] = []
  const candidates = new Map<string, Stat>()
  report.forEach((stat: Stat) => {
    if (stat.type === 'inbound-rtp' && stat.kind === 'video') video = stat
    else if (stat.type === 'transport') transport = stat
    else if (stat.type === 'candidate-pair') pairs.push(stat)
    else if (stat.type === 'local-candidate') candidates.set(String(stat.id), stat)
  })
  const pair = pairs.find((p) => p.id === transport?.selectedCandidatePairId) ??
    pairs.find((p) => p.selected === true || (p.nominated === true && p.state === 'succeeded'))

  const stats: LinkStats = {
    ...NO_STATS,
    rttMs: scale(num(pair, 'currentRoundTripTime'), 1000),
    jitterMs: scale(num(video, 'jitter'), 1000),
    width: num(video, 'frameWidth'),
    height: num(video, 'frameHeight'),
    dropped: num(video, 'framesDropped'),
    freezes: num(video, 'freezeCount'),
    nacks: num(video, 'nackCount'),
    route: route(candidates.get(String(pair?.localCandidateId))),
  }
  if (!video) return { stats, sample: null }

  const sample: Sample = {
    time: num(video, 'timestamp') ?? performance.now(),
    bytes: num(video, 'bytesReceived') ?? 0,
    received: num(video, 'packetsReceived') ?? 0,
    lost: Math.max(0, num(video, 'packetsLost') ?? 0),
    frames: num(video, 'framesDecoded') ?? 0,
    decodeTime: num(video, 'totalDecodeTime') ?? 0,
    bufferDelay: num(video, 'jitterBufferDelay') ?? 0,
    bufferCount: num(video, 'jitterBufferEmittedCount') ?? 0,
  }
  // A new connection starts its counters again: then there is nothing to compare to yet.
  if (!previous || sample.time <= previous.time || sample.bytes < previous.bytes) return { stats, sample }

  const seconds = (sample.time - previous.time) / 1000
  const packets = sample.received - previous.received + (sample.lost - previous.lost)
  const frames = sample.frames - previous.frames
  const buffered = sample.bufferCount - previous.bufferCount
  stats.bitrateKbps = ((sample.bytes - previous.bytes) * 8) / seconds / 1000
  stats.lossPercent = packets > 0 ? (Math.max(0, sample.lost - previous.lost) / packets) * 100 : null
  stats.fps = frames / seconds
  if (frames > 0 && buffered > 0) {
    stats.delayMs = ((sample.bufferDelay - previous.bufferDelay) / buffered + (sample.decodeTime - previous.decodeTime) / frames) * 1000
  }
  return { stats, sample }
}
