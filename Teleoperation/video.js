// The robot's camera on the page. The server sends one WebSocket message per H.264 frame (/api/video, see protocol.py): kind
// (1 byte: 0 config, 1 key frame, 2 other frame), capture time (8 bytes, µs since the epoch), payload.
//
// Two ways to show it, because browsers differ:
//  - WebCodecs decodes each frame as it arrives and draws it on a <canvas>: the least delay. Browsers only offer it to pages
//    on localhost or https.
//  - Media Source Extensions play it in a <video>, which needs the frames in fragmented MP4 (Mp4Muxer below). Works on plain
//    http too, so this is what a phone on the local network gets (iPhone: ManagedMediaSource, iOS 17.1 or later).
// ?video=mse or ?video=webcodecs in the page address forces one, for testing.

const CONFIG = 0, KEY = 1;
const STALE_MS = 2000;                 // no new picture for this long and the feed counts as lost
const RETRY_MS = [500, 1000, 2000, 4000];

export class VideoFeed {
  // onchange(state): "connecting" (no link to the server), "waiting" (linked, no picture), "live", "unsupported".
  constructor(video, canvas, onchange) {
    this.video = video;
    this.canvas = canvas;
    this.onchange = onchange;
    this.state = "connecting";
    this.socket = null;
    this.sink = null;
    this.configuring = null;  // frames that arrive while the decoder is being set up
    this.running = false;
    this.retries = 0;
    this.timer = null;
    this.lastShown = 0;
    this.force = new URLSearchParams(location.search).get("video");
    // For debugging (window.teleopVideo in the console). latencyMs is capture to screen; it needs the robot and the browser
    // on the same clock, so it is only meaningful when both run on one computer.
    this.stats = { mode: null, received: 0, shown: 0, latencyMs: null };
    setInterval(() => {
      if (this.state === "live" && performance.now() - this.lastShown > STALE_MS) this._set("waiting");
    }, 500);
  }

  start() {
    if (this.running) return;
    this.running = true;
    this._connect();
  }

  stop() {
    this.running = false;
    clearTimeout(this.timer);
    const socket = this.socket;
    this.socket = null;
    if (socket) socket.close();
    this._teardown();
    if (this.state !== "unsupported") this._set("connecting");
  }

  _set(state) {
    if (state === this.state) return;
    this.state = state;
    this.onchange(state);
  }

  _connect() {
    const socket = new WebSocket(`${location.protocol === "https:" ? "wss" : "ws"}://${location.host}/api/video`);
    socket.binaryType = "arraybuffer";
    this.socket = socket;
    socket.onopen = () => {
      this.retries = 0;
      this._set("waiting");
    };
    socket.onmessage = (event) => { if (this.socket === socket) this._message(event.data); };
    socket.onclose = () => {
      if (this.socket !== socket) return;
      this.socket = null;
      this._teardown();
      if (this.state === "unsupported") return;
      this._set("connecting");
      if (this.running) this.timer = setTimeout(() => this._connect(), RETRY_MS[Math.min(this.retries++, RETRY_MS.length - 1)]);
    };
  }

  _message(data) {
    if (data.byteLength < 9) return;
    const kind = new Uint8Array(data, 0, 1)[0];
    const time = Number(new DataView(data).getBigUint64(1));
    const payload = new Uint8Array(data, 9);
    if (kind === CONFIG) {
      this._configure(JSON.parse(new TextDecoder().decode(payload)));
      return;
    }
    this.stats.received++;
    if (this.configuring) this.configuring.push([payload, kind === KEY, time]);
    else if (this.sink) this.sink.frame(payload, kind === KEY, time);
  }

  async _configure(config) {
    this._teardown();
    const pending = this.configuring = [];
    const avcc = Uint8Array.from(atob(config.avcc), (c) => c.charCodeAt(0));
    const shown = (time) => this._shown(time);
    const failed = () => { if (this.sink) this._rebuild(config, avcc); };
    let sink = null;
    if (this.force !== "mse" && await CanvasSink.supported(config, avcc)) sink = new CanvasSink(this.canvas, config, avcc, shown, failed);
    else if (this.force !== "webcodecs" && VideoSink.supported(config)) sink = new VideoSink(this.video, config, avcc, shown, failed);
    if (this.configuring !== pending) { if (sink) sink.close(); return; }  // torn down meanwhile
    this.configuring = null;
    if (!sink) {
      this._set("unsupported");
      this.stop();
      return;
    }
    this._use(sink);
    for (const frame of pending) sink.frame(...frame);
  }

  _rebuild(config, avcc) {  // after a decoder error: start again from the next key frame
    const Sink = this.sink.constructor;
    this.sink.close();
    this._use(new Sink(Sink === CanvasSink ? this.canvas : this.video, config, avcc, (t) => this._shown(t),
      () => { if (this.sink) this._rebuild(config, avcc); }));
  }

  _use(sink) {
    this.sink = sink;
    this.stats.mode = sink.mode;
    this.canvas.hidden = sink.mode !== "webcodecs";
    this.video.hidden = sink.mode === "webcodecs";
  }

  _teardown() {
    this.configuring = null;
    if (this.sink) this.sink.close();
    this.sink = null;
  }

  _shown(time) {
    this.lastShown = performance.now();
    this.stats.shown++;
    if (time) this.stats.latencyMs = Math.round(Date.now() - time / 1000);
    if (this.state !== "live") this._set("live");
  }
}

function decoderConfig(config, avcc) {
  return { codec: config.codec, description: avcc, codedWidth: config.width, codedHeight: config.height, optimizeForLatency: true };
}

// WebCodecs: decode as the frames come, draw the newest one on the next screen refresh.
class CanvasSink {
  static async supported(config, avcc) {
    if (!("VideoDecoder" in window)) return false;
    try {
      return (await VideoDecoder.isConfigSupported(decoderConfig(config, avcc))).supported === true;
    } catch (error) {
      return false;
    }
  }

  constructor(canvas, config, avcc, onshown, onfailed) {
    this.mode = "webcodecs";
    this.canvas = canvas;
    this.context = canvas.getContext("2d");
    this.onshown = onshown;
    this.needKey = true;
    this.latest = null;
    this.drawing = 0;
    this.decoder = new VideoDecoder({ output: (frame) => this._output(frame), error: () => onfailed() });
    this.decoder.configure(decoderConfig(config, avcc));
  }

  frame(data, key, time) {
    if (this.decoder.state !== "configured") return;
    if (!key && (this.needKey || this.decoder.decodeQueueSize > 4)) {  // behind: wait for the next key frame
      this.needKey = true;
      return;
    }
    this.needKey = false;
    this.decoder.decode(new EncodedVideoChunk({ type: key ? "key" : "delta", timestamp: time, data }));
  }

  _output(frame) {
    if (this.latest) this.latest.close();
    this.latest = frame;
    if (!this.drawing) this.drawing = requestAnimationFrame(() => this._draw());
  }

  _draw() {
    this.drawing = 0;
    const frame = this.latest;
    this.latest = null;
    if (!frame) return;
    if (this.canvas.width !== frame.displayWidth || this.canvas.height !== frame.displayHeight) {
      this.canvas.width = frame.displayWidth;
      this.canvas.height = frame.displayHeight;
    }
    this.context.drawImage(frame, 0, 0);
    this.onshown(frame.timestamp);
    frame.close();
  }

  close() {
    cancelAnimationFrame(this.drawing);
    if (this.latest) this.latest.close();
    this.latest = null;
    if (this.decoder.state !== "closed") this.decoder.close();
  }
}

// Media Source Extensions: one MP4 fragment per frame, played as close to the newest frame as the browser allows.
class VideoSink {
  static mediaSource() {
    return window.ManagedMediaSource || window.MediaSource;
  }

  static supported(config) {
    const MediaSourceType = VideoSink.mediaSource();
    return !!MediaSourceType && MediaSourceType.isTypeSupported(`video/mp4; codecs="${config.codec}"`);
  }

  constructor(video, config, avcc, onshown, onfailed) {
    const MediaSourceType = VideoSink.mediaSource();
    this.mode = MediaSourceType === window.MediaSource ? "mse" : "managed-mse";
    this.video = video;
    this.onshown = onshown;
    this.onfailed = onfailed;
    this.muxer = new Mp4Muxer(avcc, config.width, config.height, config.fps);
    this.queue = [this.muxer.init()];
    this.times = new Array(256);  // capture time of the last frames, by frame number, for the latency figure
    this.needKey = true;
    this.buffer = null;
    this.closed = false;
    this.source = new MediaSourceType();
    video.disableRemotePlayback = true;  // ManagedMediaSource on iOS requires it
    this.url = URL.createObjectURL(this.source);
    video.src = this.url;
    this.source.addEventListener("sourceopen", () => {
      if (this.closed) return;
      this.buffer = this.source.addSourceBuffer(`video/mp4; codecs="${config.codec}"`);
      this.buffer.mode = "segments";
      this.buffer.addEventListener("updateend", () => this._appended());
      this._pump();
    }, { once: true });
    this.onerror = () => { if (!this.closed) onfailed(); };
    video.addEventListener("error", this.onerror);
    if ("requestVideoFrameCallback" in video) {
      const tick = (now, metadata) => {
        if (this.closed) return;
        this.onshown(this.times[Math.round(metadata.mediaTime * this.muxer.fps) % 256]);
        this.callback = video.requestVideoFrameCallback(tick);
      };
      this.callback = video.requestVideoFrameCallback(tick);
    } else {
      this.ontime = () => this.onshown(null);
      video.addEventListener("timeupdate", this.ontime);
    }
  }

  frame(data, key, time) {
    if (!key && (this.needKey || this.queue.length > 30)) {  // behind: wait for the next key frame
      this.needKey = true;
      return;
    }
    this.needKey = false;
    this.times[this.muxer.count % 256] = time;
    this.queue.push(this.muxer.fragment(data, key));
    this._pump();
  }

  _pump() {
    if (this.closed || !this.buffer || this.buffer.updating || !this.queue.length || this.source.readyState !== "open") return;
    const data = concat(this.queue);
    this.queue = [];
    try {
      this.buffer.appendBuffer(data);
    } catch (error) {
      this.onfailed();
    }
  }

  _appended() {
    if (this.closed) return;
    const video = this.video, ranges = this.buffer.buffered;
    if (ranges.length) {
      const start = ranges.start(ranges.length - 1), end = ranges.end(ranges.length - 1);
      // Play close to the newest frame. Far behind (the page or the network stalled): jump. A little behind: play faster
      // until caught up. Only two rates, switched with some hysteresis: setting the rate on every frame made Chrome stall.
      const lag = end - video.currentTime;
      if (video.currentTime < start || lag > 0.5) video.currentTime = Math.max(start, end - 0.05);
      else if (video.playbackRate === 1 && lag > 0.15) video.playbackRate = 1.25;
      else if (video.playbackRate !== 1 && lag < 0.08) video.playbackRate = 1;
      if (video.paused) video.play().catch(() => {});
      if (video.currentTime - ranges.start(0) > 20) {  // keep a few seconds behind the playhead, drop the rest
        this.buffer.remove(0, video.currentTime - 10);
        return;  // the removal ends with another updateend, which appends what is queued
      }
    }
    this._pump();
  }

  close() {
    this.closed = true;
    if (this.callback !== undefined) this.video.cancelVideoFrameCallback(this.callback);
    if (this.ontime) this.video.removeEventListener("timeupdate", this.ontime);
    this.video.removeEventListener("error", this.onerror);
    this.video.removeAttribute("src");
    this.video.load();
    URL.revokeObjectURL(this.url);
  }
}

function concat(parts) {
  if (parts.length === 1) return parts[0];
  const out = new Uint8Array(parts.reduce((n, p) => n + p.length, 0));
  let at = 0;
  for (const part of parts) {
    out.set(part, at);
    at += part.length;
  }
  return out;
}

// Fragmented MP4 (ISO 14496-12) with one H.264 track: init() is the header, fragment() one frame. The frames get a steady
// 1/fps timeline whatever their real timing, so the browser never sees a gap (a skipped frame just shows as a jump).
export class Mp4Muxer {
  constructor(avcc, width, height, fps) {
    this.avcc = avcc;
    this.width = width;
    this.height = height;
    this.fps = fps;
    this.timescale = 90000;
    this.duration = Math.round(this.timescale / fps);
    this.count = 0;  // frames muxed so far
  }

  init() {
    const { width, height } = this;
    const matrix = [0x10000, 0, 0, 0, 0x10000, 0, 0, 0, 0x40000000];
    const avc1 = box("avc1", zeros(6), u16(1), zeros(16), u16(width), u16(height), u32(0x480000), u32(0x480000), u32(0),
      u16(1), zeros(32), u16(0x18), u16(0xffff), box("avcC", this.avcc));
    const stbl = box("stbl", full("stsd", 0, 0, u32(1), avc1), full("stts", 0, 0, u32(0)), full("stsc", 0, 0, u32(0)),
      full("stsz", 0, 0, u32(0), u32(0)), full("stco", 0, 0, u32(0)));
    const minf = box("minf", full("vmhd", 0, 1, zeros(8)), box("dinf", full("dref", 0, 0, u32(1), full("url ", 0, 1))), stbl);
    const mdia = box("mdia", full("mdhd", 0, 0, u32(0), u32(0), u32(this.timescale), u32(0), u16(0x55c4), u16(0)),
      full("hdlr", 0, 0, u32(0), ascii("vide"), zeros(12), ascii("VideoHandler\0")), minf);
    const tkhd = full("tkhd", 0, 3, u32(0), u32(0), u32(1), u32(0), u32(0), zeros(8), u16(0), u16(0), u16(0), u16(0),
      ...matrix.map(u32), u32(width << 16), u32(height << 16));
    const mvhd = full("mvhd", 0, 0, u32(0), u32(0), u32(1000), u32(0), u32(0x10000), u16(0x100), zeros(10),
      ...matrix.map(u32), zeros(24), u32(2));
    const mvex = box("mvex", full("trex", 0, 0, u32(1), u32(1), u32(0), u32(0), u32(0)));
    return concat([box("ftyp", ascii("iso5"), u32(512), ascii("iso5"), ascii("iso6"), ascii("mp41")),
      box("moov", mvhd, box("trak", tkhd, mdia), mvex)]);
  }

  fragment(sample, key) {
    this.count++;
    const time = (this.count - 1) * this.duration;
    const flags = key ? 0x02000000 : 0x01010000;  // depends on no other frame / depends on others, not a sync sample
    const trun = (offset) => full("trun", 0, 0x000701, u32(1), u32(offset), u32(this.duration), u32(sample.length), u32(flags));
    const traf = (offset) => box("traf", full("tfhd", 0, 0x020000, u32(1)),
      full("tfdt", 1, 0, u32(Math.floor(time / 2 ** 32)), u32(time >>> 0)), trun(offset));
    const size = box("moof", full("mfhd", 0, 0, u32(this.count)), traf(0)).length;
    const moof = box("moof", full("mfhd", 0, 0, u32(this.count)), traf(size + 8));  // data offset: from the moof to the sample
    return concat([moof, box("mdat", sample)]);
  }
}

function box(type, ...parts) {
  const size = 8 + parts.reduce((n, p) => n + p.length, 0);
  const out = new Uint8Array(size);
  new DataView(out.buffer).setUint32(0, size);
  out.set(ascii(type), 4);
  let at = 8;
  for (const part of parts) {
    out.set(part, at);
    at += part.length;
  }
  return out;
}

function full(type, version, flags, ...parts) {
  return box(type, new Uint8Array([version, flags >> 16, (flags >> 8) & 255, flags & 255]), ...parts);
}

function u32(value) {
  const out = new Uint8Array(4);
  new DataView(out.buffer).setUint32(0, value >>> 0);
  return out;
}

function u16(value) {
  return new Uint8Array([(value >> 8) & 255, value & 255]);
}

function zeros(n) {
  return new Uint8Array(n);
}

function ascii(text) {
  return Uint8Array.from(text, (c) => c.charCodeAt(0));
}
