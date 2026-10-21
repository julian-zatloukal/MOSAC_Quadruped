"""The page's side of the Teleoperation server: one WebRTC connection per page, carrying the camera to it and the commands and
the robot's status both ways.

    page --- POST /api/offer (the only HTTP besides the page itself) ---> Pages.answer() --- answer --> page
    page <== video track: the camera, H.264 as the robot encoded it ====== RelayTrack <-- VideoRelay (TCP from the robot)
    page <=> data channel "teleop": commands from the page, the state back every STATUS_PERIOD ==> RobotLink (UDP)

The messages on the data channel are in protocol.py. asyncio (aiortc) runs in its own thread, so the rest of the server stays
plain threads and sockets: Pages.answer() blocks until the answer is ready, and the video arrives from the relay thread through
RelayTrack.put().

Why it is built like this:
 - The video is not decoded and encoded again. aiortc's sender takes H.264 packets that are already encoded (a track whose recv()
   returns av.Packet) and only cuts them into RTP packets, so the picture costs this process almost nothing and gains no delay.
   The price is that a page's request for a new key frame (PLI) cannot reach the encoder; the robot sends one every second.
 - The data channel is ordered but gives up on a message after COMMAND_LIFETIME_MS (set by the page): a command that could not
   be delivered in time is useless, and waiting for it would hold up the newer ones behind it. A lost stop is covered by the
   robot, which stops by itself when the drive commands stop.
 - No STUN or TURN: the robot is on this computer or its network, and nothing leaves it.
"""
import asyncio
import base64
import json
import struct
import threading
import time

import av
from aiortc import RTCConfiguration, RTCPeerConnection, RTCRtpSender, RTCSessionDescription
from aiortc.exceptions import InvalidStateError
from aiortc.mediastreams import VIDEO_TIME_BASE, MediaStreamError, MediaStreamTrack

import protocol

CHANNEL_LABEL = "teleop"
MAX_PAGES = 8          # connections at once; each one costs CPU while the camera is on
ANSWER_TIMEOUT = 10.0  # s
CONNECT_TIMEOUT = 15.0  # s a page has to finish connecting after the answer, or it is dropped
START_CODE = b"\x00\x00\x00\x01"

def parameter_sets(avcc):
    """(SPS and PPS as NAL units with start codes, size of the length fields of the frames) from an H.264 decoder configuration
    record (ISO 14496-15)."""
    nal_units, pos = bytearray(), 5
    for mask in (0x1F, 0xFF):  # a count (5 bits for the SPS), then that many (length, NAL unit)
        count, pos = avcc[pos] & mask, pos + 1
        for _ in range(count):
            size = int.from_bytes(avcc[pos:pos + 2], "big")
            nal_units += START_CODE + avcc[pos + 2:pos + 2 + size]
            pos += 2 + size
    return bytes(nal_units), (avcc[4] & 3) + 1


def annex_b(frame, length_size):
    """A frame's NAL units with start codes, from NAL units that follow a length field (AVCC)."""
    nal_units, pos = bytearray(), 0
    while pos + length_size <= len(frame):
        size = int.from_bytes(frame[pos:pos + length_size], "big")
        nal_units += START_CODE + frame[pos + length_size:pos + length_size + size]
        pos += length_size + size
    return bytes(nal_units)


class RelayTrack(MediaStreamTrack):
    """The camera as a video track, fed by VideoRelay (put() from its thread). The queue is a protocol.VideoQueue, so a page
    that falls behind skips to the next key frame instead of building up delay."""

    kind = "video"

    def __init__(self, loop):
        super().__init__()
        self._loop = loop
        self._queue = protocol.VideoQueue()
        self._ready = asyncio.Event()
        self._headers = b""     # SPS and PPS: browsers need them in front of every key frame
        self._length_size = 4

    def put(self, message):
        self._queue.put(message)
        self._wake()

    def close(self):
        self._queue.close()
        self._wake()

    def stop(self):
        super().stop()
        self._queue.close()

    def _wake(self):
        try:
            self._loop.call_soon_threadsafe(self._ready.set)
        except RuntimeError:  # the loop is gone: the server is shutting down
            pass

    async def recv(self):
        while True:
            message = self._queue.get(timeout=0)
            if message is None:
                if self._queue.closed or self.readyState != "live":
                    raise MediaStreamError
                self._ready.clear()
                await self._ready.wait()
                continue
            kind, stamp = message[4], struct.unpack_from(">Q", message, 5)[0]
            payload = message[protocol.VIDEO_HEADER.size:]
            if kind == protocol.VIDEO_CONFIG:
                self._headers, self._length_size = parameter_sets(base64.b64decode(json.loads(payload)["avcc"]))
                continue
            packet = av.Packet((self._headers if kind == protocol.VIDEO_KEY else b"") + annex_b(payload, self._length_size))
            packet.pts = stamp * 9 // 100  # the capture time, on the 90 kHz clock of the RTP timestamps
            packet.time_base = VIDEO_TIME_BASE
            return packet


class Page:
    """One connected page."""

    def __init__(self, pages):
        self.pages = pages
        self.pc = RTCPeerConnection(RTCConfiguration(iceServers=[]))
        self.track = RelayTrack(pages.loop)
        self.last_drive = None  # when the page last sent a drive command, None after a stop
        self.closed = False
        self.pc.on("datachannel", self._on_channel)
        self.pc.on("connectionstatechange", self._on_state)

    async def answer(self, sdp, kind):
        """The answer to the page's offer. The offer has to ask for video (a receive-only transceiver) for there to be any, and
        for a data channel, which is how the page drives the robot (aiortc answers an SDP without any media as if it were fine)."""
        if "m=application" not in sdp:
            raise ValueError("the offer has no data channel")
        if "m=video" in sdp:
            # Before the offer is applied: the codecs are chosen then, and H.264 is the only one the track can be sent as.
            transceiver = self.pc.addTransceiver(self.track, direction="sendonly")
            transceiver.setCodecPreferences([c for c in RTCRtpSender.getCapabilities("video").codecs
                                             if c.mimeType in ("video/H264", "video/rtx")])
            self.pages.video.subscribe(self.track)
        await self.pc.setRemoteDescription(RTCSessionDescription(sdp, kind))
        await self.pc.setLocalDescription(await self.pc.createAnswer())
        self.pages.loop.call_later(CONNECT_TIMEOUT, self._expire)
        return {"sdp": self.pc.localDescription.sdp, "type": self.pc.localDescription.type}

    async def close(self):
        if self.closed:
            return
        self.closed = True
        self.pages.discard(self)
        self.pages.video.unsubscribe(self.track)
        if self.last_drive is not None and time.monotonic() - self.last_drive < protocol.COMMAND_TIMEOUT:
            self.pages.link.send(protocol.encode_command("stop"))  # do not wait for the robot's own timeout
        await self.pc.close()

    def _expire(self):
        if self.pc.connectionState != "connected":
            asyncio.ensure_future(self.close())

    def _on_state(self):
        if self.pc.connectionState in ("failed", "closed"):
            asyncio.ensure_future(self.close())

    def _on_channel(self, channel):
        if channel.label != CHANNEL_LABEL:
            return
        channel.on("message", self._on_message)
        channel.on("close", lambda: asyncio.ensure_future(self.close()))
        asyncio.ensure_future(self._push_state(channel))

    def _on_message(self, message):
        command = protocol.decode_command(message) if isinstance(message, str) else None
        if command is None:
            return
        self.last_drive = time.monotonic() if command[0] == "drive" else None
        self.pages.link.send(protocol.encode_command(*command))

    async def _push_state(self, channel):
        while channel.readyState == "open" and not self.closed:
            try:
                channel.send(json.dumps(dict(self.pages.link.state(), video=self.pages.video.state())))
            except Exception:  # the channel closed under us
                return
            await asyncio.sleep(protocol.STATUS_PERIOD)


class Busy(Exception):
    pass


def _loop_exception(loop, context):
    """aiortc connects in a task of its own. A page that leaves (or is dropped for taking too long) while that task is starting
    closes the transport under it, and the task ends with InvalidStateError: "RTCIceTransport is closed", with nobody left to
    retrieve it. That is a page going away, not an error; anything else goes to asyncio's usual report."""
    error = context.get("exception")
    if isinstance(error, InvalidStateError) and "closed" in str(error):
        return
    loop.default_exception_handler(context)


class Pages:
    """The pages connected to the server."""

    def __init__(self, link, video):
        """link: the RobotLink for the commands and the state. video: the VideoRelay."""
        self.link, self.video = link, video
        self.loop = asyncio.new_event_loop()
        self.loop.set_exception_handler(_loop_exception)
        self._pages = set()
        self._thread = threading.Thread(target=self.loop.run_forever, daemon=True)
        self._thread.start()

    def answer(self, sdp, kind):
        """{"sdp", "type"} answering a page's offer. Blocks. Raises Busy when MAX_PAGES pages are connected, and ValueError or
        whatever aiortc raises for an offer it cannot use."""
        return asyncio.run_coroutine_threadsafe(self._answer(sdp, kind), self.loop).result(ANSWER_TIMEOUT)

    async def _answer(self, sdp, kind):
        if len(self._pages) >= MAX_PAGES:
            raise Busy
        page = Page(self)
        self._pages.add(page)
        try:
            return await page.answer(sdp, kind)
        except BaseException:
            await page.close()
            raise

    def discard(self, page):
        self._pages.discard(page)

    def close(self):
        asyncio.run_coroutine_threadsafe(self._close(), self.loop).result(5)
        self.loop.call_soon_threadsafe(self.loop.stop)
        self._thread.join(timeout=2)

    async def _close(self):
        await asyncio.gather(*(page.close() for page in list(self._pages)))
