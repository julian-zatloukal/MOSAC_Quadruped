import { useEffect, useState } from 'react'
import { NO_STATS, readStats, type LinkStats, type Sample } from '../teleop/stats.ts'

const FIRST_MS = 500 // the second reading, which the rates need
const EVERY_MS = 1000

// The link statistics, read once a second while `active` (the details are open): nothing is measured for a pill nobody opened.
export function useLinkStats(getStats: () => Promise<RTCStatsReport> | null, active: boolean): LinkStats {
  const [stats, setStats] = useState<LinkStats>(NO_STATS)

  useEffect(() => {
    if (!active) return
    let sample: Sample | null = null
    let timer: number | undefined
    let stopped = false
    const poll = async () => {
      let report: RTCStatsReport | null = null
      try {
        report = await getStats()
      } catch {
        /* the connection closed meanwhile */
      }
      if (stopped) return
      if (report) {
        const next = readStats(report, sample)
        sample = next.sample
        setStats(next.stats)
      } else {
        sample = null
        setStats(NO_STATS)
      }
      timer = setTimeout(poll, sample ? EVERY_MS : FIRST_MS)
    }
    void poll()
    return () => {
      stopped = true
      clearTimeout(timer)
    }
  }, [getStats, active])

  return active ? stats : NO_STATS
}
