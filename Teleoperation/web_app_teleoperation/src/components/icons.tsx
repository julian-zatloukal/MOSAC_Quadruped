import type { ReactNode } from 'react'

function Icon({ className, children }: { className?: string; children: ReactNode }) {
  return (
    <svg className={className ? `icon ${className}` : 'icon'} viewBox="0 0 24 24" aria-hidden="true">
      {children}
    </svg>
  )
}

export const ForwardIcon = () => <Icon><path d="M12 19V5M5.5 11.5L12 5l6.5 6.5" /></Icon>
export const BackIcon = () => <Icon><path d="M12 5v14M5.5 12.5L12 19l6.5-6.5" /></Icon>
export const YawLeftIcon = () => <Icon><path d="M3 12a9 9 0 1 0 9-9 9.75 9.75 0 0 0-6.74 2.74L3 8" /><path d="M3 3v5h5" /></Icon>
export const YawRightIcon = () => <Icon><path d="M21 12a9 9 0 1 1-9-9c2.52 0 4.93 1 6.74 2.74L21 8" /><path d="M21 3v5h-5" /></Icon>

export const NoVideoIcon = () => (
  <Icon>
    <path d="M16 16v1a2 2 0 0 1-2 2H4a2 2 0 0 1-2-2V7a2 2 0 0 1 2-2h2m5.7 0H14a2 2 0 0 1 2 2v3.3l1 1 5-3.3v8" />
    <path d="M2 2l20 20" />
  </Icon>
)

// The needle points to the heading; counter-clockwise is positive, as in the world.
export const CompassIcon = ({ yaw }: { yaw: number }) => (
  <svg className="compass" viewBox="0 0 24 24" aria-hidden="true">
    <circle cx="12" cy="12" r="10" />
    <path d="M12 4.5l4 10h-8z" style={{ transform: `rotate(${-yaw}deg)` }} />
  </svg>
)

export const FitIcon = ({ fit }: { fit: boolean }) => (
  <Icon><path d={fit ? 'M15 3h6v6M9 21H3v-6M21 3l-7 7M3 21l7-7' : 'M4 14h6v6M20 10h-6V4M14 10l7-7M3 21l7-7'} /></Icon>
)

export const FullscreenIcon = ({ active }: { active: boolean }) => (
  <Icon>
    <path d={active
      ? 'M8 3v3a2 2 0 0 1-2 2H3M21 8h-3a2 2 0 0 1-2-2V3M3 16h3a2 2 0 0 1 2 2v3M16 21v-3a2 2 0 0 1 2-2h3'
      : 'M8 3H5a2 2 0 0 0-2 2v3M21 8V5a2 2 0 0 0-2-2h-3M3 16v3a2 2 0 0 0 2 2h3M16 21h3a2 2 0 0 0 2-2v-3'} />
  </Icon>
)
