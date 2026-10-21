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

// Safari before 16.4 has the prefixed Fullscreen API only. The iPhone's Safari has none for a page (WebKit bug 206854 is still open),
// so the button is not shown there; Android's Chrome, iPad and computers have it.
interface PrefixedDocument extends Document {
  webkitFullscreenEnabled?: boolean
  webkitFullscreenElement?: Element | null
  webkitExitFullscreen?: () => void
}
interface PrefixedElement extends HTMLElement {
  webkitRequestFullscreen?: () => Promise<void> | void
}

const doc = document as PrefixedDocument
const root = document.documentElement as PrefixedElement
const isFullscreen = () => !!(doc.fullscreenElement ?? doc.webkitFullscreenElement)
const request = root.requestFullscreen ?? root.webkitRequestFullscreen
// The button is shown when the browser can do it here: the method exists and the browser does not say it is off (it does, for
// one, in a frame that is not allowed to), and the page is not an installed app that already starts full screen (display:
// fullscreen in the manifest that server.py serves).
const capable = !!request && (doc.fullscreenEnabled ?? doc.webkitFullscreenEnabled) !== false
const installedFullscreen = matchMedia('(display-mode: fullscreen)').matches

// Full screen, where the browser has it.
export function useFullscreen() {
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
    if (isFullscreen()) (doc.exitFullscreen ?? doc.webkitExitFullscreen)?.call(doc)
    else request?.call(root)?.catch?.(() => {}) // the browser may say no
  }
  return { available: capable && !installedFullscreen, active, toggle }
}
