import { useEffect, useState } from 'react'
import { DriveInput, type DriveRole } from '../teleop/driveInput.ts'
import type { Command } from '../teleop/protocol.ts'

const KEYS: Record<string, DriveRole> = {
  ArrowUp: 'forward', KeyW: 'forward', ArrowDown: 'back', KeyS: 'back',
  ArrowLeft: 'left', KeyA: 'left', ArrowRight: 'right', KeyD: 'right',
}

// The drive controls: the buttons call press() and release(), the keyboard is handled here. Letting go of the page must never
// leave the robot moving, so everything is released when the window loses focus or the tab is hidden.
export function useDrive(send: (command: Command) => unknown) {
  const [held, setHeld] = useState<ReadonlySet<DriveRole>>(() => new Set())
  const [input] = useState(() => new DriveInput(send, setHeld))

  useEffect(() => {
    const keyDown = (event: KeyboardEvent) => {
      if (event.repeat || event.ctrlKey || event.metaKey || event.altKey) return
      if (event.code in KEYS) {
        event.preventDefault()
        input.press(event.code, KEYS[event.code])
      }
    }
    const keyUp = (event: KeyboardEvent) => {
      if (event.code in KEYS) input.release(event.code)
    }
    const letGo = () => input.releaseAll()
    const hidden = () => {
      if (document.hidden) input.releaseAll()
    }
    const noMenu = (event: Event) => event.preventDefault() // a long press must not open a menu

    addEventListener('keydown', keyDown)
    addEventListener('keyup', keyUp)
    addEventListener('blur', letGo)
    addEventListener('pagehide', letGo)
    addEventListener('contextmenu', noMenu)
    document.addEventListener('visibilitychange', hidden)
    return () => {
      removeEventListener('keydown', keyDown)
      removeEventListener('keyup', keyUp)
      removeEventListener('blur', letGo)
      removeEventListener('pagehide', letGo)
      removeEventListener('contextmenu', noMenu)
      document.removeEventListener('visibilitychange', hidden)
      input.releaseAll()
    }
  }, [input])

  return {
    held,
    press: (id: string, role: DriveRole) => input.press(id, role),
    release: (id: string) => input.release(id),
  }
}
