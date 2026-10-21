import { useEffect, useState } from 'react'

const FIT_KEY = 'teleop.fit'

function readFit(): boolean {
  try {
    return localStorage.getItem(FIT_KEY) === '1'
  } catch {
    return false // private mode
  }
}

// Fill the screen with the picture (cropped), or show all of it. Remembered for next time.
export function useFit() {
  const [fit, setFit] = useState(readFit)
  const toggle = () => {
    const next = !fit
    setFit(next)
    try {
      localStorage.setItem(FIT_KEY, next ? '1' : '0')
    } catch {
      /* private mode */
    }
  }
  return { fit, toggle }
}

// Safari before 16.4 has the prefixed Fullscreen API only (and the iPhone none at all).
interface PrefixedDocument extends Document {
  webkitFullscreenEnabled?: boolean
  webkitFullscreenElement?: Element | null
  webkitExitFullscreen?: () => void
}
interface PrefixedElement extends HTMLElement {
  webkitRequestFullscreen?: () => void
}

const doc = document as PrefixedDocument
const isFullscreen = () => !!(doc.fullscreenElement ?? doc.webkitFullscreenElement)

// Full screen, where the browser allows it.
export function useFullscreen() {
  const supported = !!(doc.fullscreenEnabled || doc.webkitFullscreenEnabled)
  const [active, setActive] = useState(isFullscreen)

  useEffect(() => {
    const update = () => setActive(isFullscreen())
    document.addEventListener('fullscreenchange', update)
    document.addEventListener('webkitfullscreenchange', update)
    return () => {
      document.removeEventListener('fullscreenchange', update)
      document.removeEventListener('webkitfullscreenchange', update)
    }
  }, [])

  const toggle = () => {
    const root = document.documentElement as PrefixedElement
    if (isFullscreen()) (doc.exitFullscreen ?? doc.webkitExitFullscreen)?.call(doc)
    else (root.requestFullscreen ?? root.webkitRequestFullscreen)?.call(root)
  }
  return { supported, active, toggle }
}
