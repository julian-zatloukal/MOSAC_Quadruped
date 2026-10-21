import type { PointerEvent, ReactNode } from 'react'
import type { DriveRole } from '../teleop/driveInput.ts'
import { BackIcon, ForwardIcon, YawLeftIcon, YawRightIcon } from './icons.tsx'
import './Controls.css'

interface DriveKeyProps {
  role: DriveRole
  label: string
  active: boolean
  onPress: (id: string, role: DriveRole) => void
  onRelease: (id: string) => void
  children: ReactNode
}

// A button that drives while it is held. Pointer events cover mouse, touch and pen; the button keeps the pointer until it is
// released, wherever it moves meanwhile.
function DriveKey({ role, label, active, onPress, onRelease, children }: DriveKeyProps) {
  const release = (event: PointerEvent) => onRelease(`pointer${event.pointerId}`)
  return (
    <button
      className={active ? 'key glass active' : 'key glass'}
      id={role}
      type="button"
      aria-label={label}
      aria-pressed={active}
      onPointerDown={(event) => {
        if (event.pointerType === 'mouse' && event.button !== 0) return
        event.currentTarget.setPointerCapture(event.pointerId)
        onPress(`pointer${event.pointerId}`, role)
      }}
      onPointerUp={release}
      onPointerCancel={release}
      onLostPointerCapture={release}
    >
      {children}
    </button>
  )
}

interface ControlsProps {
  held: ReadonlySet<DriveRole>
  onPress: (id: string, role: DriveRole) => void
  onRelease: (id: string) => void
}

// Forward and back under the left thumb, yaw left and right under the right. The robot stops when they are let go.
export function Controls({ held, onPress, onRelease }: ControlsProps) {
  const key = (role: DriveRole) => ({ role, active: held.has(role), onPress, onRelease })
  return (
    <footer className="controls">
      <div className="pad drive">
        <DriveKey {...key('forward')} label="Forward"><ForwardIcon /></DriveKey>
        <DriveKey {...key('back')} label="Back"><BackIcon /></DriveKey>
      </div>
      <div className="pad yaw">
        <DriveKey {...key('left')} label="Yaw left"><YawLeftIcon /></DriveKey>
        <DriveKey {...key('right')} label="Yaw right"><YawRightIcon /></DriveKey>
      </div>
    </footer>
  )
}
