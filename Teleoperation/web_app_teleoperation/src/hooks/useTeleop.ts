import { useEffect, useState } from 'react'
import { TeleopConnection, type Link } from '../teleop/connection.ts'
import type { ServerState } from '../teleop/protocol.ts'

// The connection to the server, kept for as long as the page is visible: a hidden tab lets go of the camera, and the server
// switches it off a few seconds later.
export function useTeleop() {
  const [link, setLink] = useState<Link>('connecting')
  const [state, setState] = useState<ServerState | null>(null)
  const [stream, setStream] = useState<MediaStream | null>(null)
  const [connection] = useState(() => new TeleopConnection({ onLink: setLink, onState: setState, onStream: setStream }))

  useEffect(() => {
    const follow = () => (document.hidden ? connection.stop() : connection.start())
    follow()
    document.addEventListener('visibilitychange', follow)
    return () => {
      document.removeEventListener('visibilitychange', follow)
      connection.stop()
    }
  }, [connection])

  return { link, robot: state?.robot ?? null, stream, send: connection.send, getStats: connection.getStats }
}
