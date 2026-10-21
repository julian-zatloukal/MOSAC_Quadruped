import { HEARTBEAT_MS, type Command } from './protocol.ts'

export type DriveRole = 'forward' | 'back' | 'left' | 'right'

// What is held down (buttons, touches and keys, any number at once, like WASD), turned into commands: one when something
// changes, and a repeat every HEARTBEAT_MS while anything is held. Always at the full stride and turn; letting go stops.
export class DriveInput {
  private send: (command: Command) => unknown
  private onHeld: (held: ReadonlySet<DriveRole>) => void
  private held = new Map<string, DriveRole>() // the pointer or key -> what it asks for
  private timer: number | undefined

  constructor(send: (command: Command) => unknown, onHeld: (held: ReadonlySet<DriveRole>) => void) {
    this.send = send
    this.onHeld = onHeld
  }

  press(id: string, role: DriveRole) {
    if (this.held.get(id) === role) return
    this.held.set(id, role)
    this.publish()
    this.send(this.command())
    this.timer ??= setInterval(() => this.send(this.command()), HEARTBEAT_MS)
  }

  release(id: string) {
    if (!this.held.delete(id)) return // not held
    this.publish()
    if (this.held.size === 0) {
      this.stopRepeating()
      this.send({ cmd: 'stop' })
    } else {
      this.send(this.command())
    }
  }

  // Lets go of everything, and stops if anything was held.
  releaseAll() {
    const wasHeld = this.held.size > 0
    this.held.clear()
    this.stopRepeating()
    this.publish()
    if (wasHeld) this.send({ cmd: 'stop' })
  }

  private command(): Command {
    const roles = new Set(this.held.values())
    const forward = (roles.has('forward') ? 1 : 0) - (roles.has('back') ? 1 : 0)
    const turn = (roles.has('left') ? 1 : 0) - (roles.has('right') ? 1 : 0)
    return forward === 0 && turn === 0 ? { cmd: 'stop' } : { cmd: 'drive', forward, turn }
  }

  private stopRepeating() {
    clearInterval(this.timer)
    this.timer = undefined
  }

  private publish() {
    this.onHeld(new Set(this.held.values()))
  }
}
