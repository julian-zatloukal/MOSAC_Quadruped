import { useEffect, useState, type RefObject } from 'react'

// 'waiting': connected, no picture yet. 'live': pictures are arriving. 'stalled': none for SLOW_START_MS after connecting.
export type VideoPhase = 'waiting' | 'live' | 'stalled'

const STALE_MS = 2000 // no new picture for this long and the feed counts as lost
const SLOW_START_MS = 3000 // how long "Connecting" may last before the page says there is no video
const CHECK_MS = 250

// Shows the stream in the <video> and says whether pictures are really arriving.
export function useVideoPhase(videoRef: RefObject<HTMLVideoElement | null>, stream: MediaStream | null): VideoPhase {
  const [phase, setPhase] = useState<VideoPhase>('waiting')

  useEffect(() => {
    const video = videoRef.current
    if (!video) return
    video.srcObject = stream

    const since = performance.now()
    let lastShown = -Infinity
    let live = false
    const check = () => {
      const now = performance.now()
      live = now - lastShown < STALE_MS
      setPhase(live ? 'live' : now - since > SLOW_START_MS ? 'stalled' : 'waiting')
    }
    const shown = () => {
      lastShown = performance.now()
      if (!live) check()
    }

    // Each picture put on the screen; browsers without requestVideoFrameCallback only tell as the playback time moves.
    let callback = 0
    const onFrame = () => {
      shown()
      callback = video.requestVideoFrameCallback(onFrame)
    }
    if (typeof video.requestVideoFrameCallback === 'function') callback = video.requestVideoFrameCallback(onFrame)
    else video.addEventListener('timeupdate', shown)

    // Some phones (iOS in Low Power Mode) do not start a video until the page is touched.
    const start = () => {
      if (video.paused && video.srcObject) video.play().catch(() => {})
    }
    addEventListener('pointerdown', start)

    const timer = setInterval(check, CHECK_MS)
    return () => {
      clearInterval(timer)
      removeEventListener('pointerdown', start)
      video.removeEventListener('timeupdate', shown)
      if (callback) video.cancelVideoFrameCallback(callback)
      video.srcObject = null
    }
  }, [videoRef, stream])

  return phase
}
