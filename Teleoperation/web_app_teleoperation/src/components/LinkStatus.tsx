import { useEffect, useRef, useState } from 'react'
import { useLinkStats } from '../hooks/useLinkStats.ts'
import type { Link } from '../teleop/connection.ts'
import type { RobotMode, RobotStatus } from '../teleop/protocol.ts'
import type { LinkStats } from '../teleop/stats.ts'
import { CompassIcon } from './icons.tsx'
import './LinkStatus.css'

type Tone = 'ok' | 'warn' | 'bad'

function summary(link: Link, robot: RobotStatus | null): { tone: Tone; label: string } {
  if (link === 'down') return { tone: 'bad', label: 'Offline' }
  if (link === 'connecting') return { tone: 'warn', label: 'Connecting' }
  return robot ? { tone: 'ok', label: 'Online' } : { tone: 'warn', label: 'No robot' }
}

const MODES: Partial<Record<RobotMode, string>> = { walk: 'Walking', back: 'Reversing', turn: 'Turning' }

const modeText = (robot: RobotStatus) => MODES[robot.mode] ?? (robot.idle ? 'Standing' : 'Stopping')

// How bad a value is: past `warn` it is amber, past `bad` red.
const tone = (value: number | null, warn: number, bad: number, below = false): Tone | undefined => {
  if (value === null) return undefined
  const [w, b] = below ? [value < warn, value < bad] : [value > warn, value > bad]
  return b ? 'bad' : w ? 'warn' : undefined
}

const MISSING = '—'
const ms = (value: number | null) => {
  if (value === null) return MISSING
  if (value < 1) return '<1 ms'
  const rounded = value < 9.95 ? Math.round(value * 10) / 10 : Math.round(value)
  return `${rounded < 10 ? rounded.toFixed(1) : rounded} ms`
}
const count = (value: number | null) => (value === null ? MISSING : String(value))
const bitrate = (kbps: number | null) =>
  kbps === null ? MISSING : kbps >= 1000 ? `${(kbps / 1000).toFixed(1)} Mb/s` : `${Math.round(kbps)} kb/s`

function Metric({ label, value, tone }: { label: string; value: string; tone?: Tone }) {
  return (
    <div className="metric">
      <dt>{label}</dt>
      <dd className={tone}>{value}</dd>
    </div>
  )
}

function Metrics({ stats }: { stats: LinkStats }) {
  const { rttMs, bitrateKbps, lossPercent, jitterMs, fps, width, height, delayMs, dropped, freezes, nacks } = stats
  return (
    <dl className="metrics">
      <Metric label="Latency" value={ms(rttMs)} tone={tone(rttMs, 80, 200)} />
      <Metric label="Bitrate" value={bitrate(bitrateKbps)} />
      <Metric label="Loss" value={lossPercent === null ? MISSING : `${lossPercent.toFixed(lossPercent < 10 ? 1 : 0)} %`} tone={tone(lossPercent, 1, 5)} />
      <Metric label="Jitter" value={ms(jitterMs)} tone={tone(jitterMs, 30, 80)} />
      <Metric label="Frame rate" value={fps === null ? MISSING : `${Math.round(fps)} fps`} tone={tone(fps, 24, 15, true)} />
      <Metric label="Resolution" value={width && height ? `${width}×${height}` : MISSING} />
      <Metric label="Video delay" value={ms(delayMs)} tone={tone(delayMs, 150, 300)} />
      <Metric label="Freezes" value={count(freezes)} tone={freezes ? 'warn' : undefined} />
      <Metric label="Dropped" value={count(dropped)} />
      <Metric label="Retransmits" value={count(nacks)} />
    </dl>
  )
}

interface LinkStatusProps {
  link: Link
  robot: RobotStatus | null
  getStats: () => Promise<RTCStatsReport> | null
}

// Collapsed: whether the robot is reachable, as a dot and a word. Tapped: the robot's heading and what it is doing, and how good
// the link is.
export function LinkStatus({ link, robot, getStats }: LinkStatusProps) {
  const [open, setOpen] = useState(false)
  const root = useRef<HTMLDivElement>(null)
  const stats = useLinkStats(getStats, open)
  const { tone: dot, label } = summary(link, robot)

  // Tapping anywhere else closes it.
  useEffect(() => {
    if (!open) return
    const away = (event: PointerEvent) => {
      if (!root.current?.contains(event.target as Node)) setOpen(false)
    }
    addEventListener('pointerdown', away)
    return () => removeEventListener('pointerdown', away)
  }, [open])

  const yaw = robot?.yaw_deg ?? 0
  return (
    <div className="link glass" data-open={open} ref={root}>
      <button className="link-head" type="button" aria-expanded={open} aria-controls="link-details" onClick={() => setOpen(!open)}>
        <span className={`dot ${dot}`} />
        <span>{label}</span>
      </button>
      <div className="link-details" id="link-details" inert={!open}>
        <div className="link-body">
          <div className="link-inner">
          <div className="robot">
            <CompassIcon yaw={yaw} />
            <span className="num">{robot ? `${yaw < -0.5 ? '−' : ''}${Math.abs(Math.round(yaw))}°` : MISSING}</span>
            <span className="mode">{robot ? modeText(robot) : MISSING}</span>
          </div>
          <Metrics stats={stats} />
          <p className="route">{stats.route ?? MISSING}</p>
          </div>
        </div>
      </div>
    </div>
  )
}
