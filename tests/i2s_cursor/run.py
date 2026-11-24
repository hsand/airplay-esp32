#!/usr/bin/env python3
"""Model of ESP-IDF's I2S TX driver (esp_driver_i2s/i2s_common.c, v6.1) and the
output cursor in main/audio/audio_output.c: does the depth the scheduler uses
match when a written frame is really played?

The driver: a circular ring of DESC descriptors of FRAME frames; the DMA's EOF
interrupt calls on_sent, then pushes the descriptor it just played onto a queue
of DESC - 1 (dropping the oldest when full); i2s_channel_write() takes a new
buffer from that queue when the current one is full, unset, or the queue has
at most one space left.  After i2s_channel_enable() the queue is empty and the
DMA starts at desc[0].

For every block the playback task writes, the cursor's prediction (written at
the moment the block's first frame is computed, i.e. before the write blocks)
is compared with the instant the DMA actually plays that frame.

    python3 tests/i2s_cursor/run.py
"""
import collections
import sys

DESC, FRAME, BLOCK = 8, 256, 352


class Sim:
    def __init__(self, fixed):
        self.fixed = fixed
        self.t = 0                       # frames since enable
        self.ring = [[None] * FRAME for _ in range(DESC)]
        self.q = collections.deque()     # free buffers (indices)
        self.cur, self.rw = None, 0
        self.played = {}                 # tag -> play time
        # cursor, as audio_output.c sets it up at i2s_channel_enable()
        self.submitted = self.sent = self.lost = 0
        self.eofs = 0
        self.sent_t = 0 if fixed else None        # output_channel_enabled()
        if fixed:
            self.lost = self.ahead_of_new_writer()

    # -- DMA -----------------------------------------------------------------
    def advance(self, n):
        for _ in range(n):
            d, o = (self.t // FRAME) % DESC, self.t % FRAME
            tag = self.ring[d][o]
            if tag is not None and tag not in self.played:
                self.played[tag] = self.t
            self.ring[d][o] = None if o else self.ring[d][o]
            self.t += 1
            if self.t % FRAME == 0:                     # EOF of descriptor d
                self.ring[d] = [None] * FRAME           # auto_clear
                self.sent += FRAME
                self.sent_t = self.t
                self.eofs = min(self.eofs + 1, DESC)
                if len(self.q) == DESC - 1:
                    self.q.popleft()
                self.q.append(d)

    def until_eof(self):
        self.advance(FRAME - self.t % FRAME)

    # -- cursor (audio_output.c) ----------------------------------------------
    def ahead_of_new_writer(self):
        return (DESC - min(DESC - 1, self.eofs)) * FRAME

    def queued(self):
        if self.sent > self.submitted + self.lost:
            ahead = self.ahead_of_new_writer() if self.fixed else 0
            self.lost = self.sent - self.submitted + ahead
            return ahead
        return min(self.submitted + self.lost - self.sent, DESC * FRAME)

    def pipeline(self):
        q = self.queued()
        if self.sent_t is not None:
            q = max(0, q - min(self.t - self.sent_t, FRAME))
        return q

    def restart_writer(self):
        """audio_output_start() on a ring that kept running."""
        self.submitted = self.sent = 0
        self.lost = self.ahead_of_new_writer() if self.fixed else 0
        self.cur, self.rw = None, 0

    # -- writer (i2s_channel_write) -------------------------------------------
    def write(self, tags):
        # One memcpy per iteration, as the driver does: the "queue almost
        # full" test runs per chunk, not per frame.
        i = 0
        while i < len(tags):
            if self.rw == FRAME or self.cur is None or DESC - 1 - len(self.q) <= 1:
                while not self.q:
                    self.until_eof()
                self.cur, self.rw = self.q.popleft(), 0
            n = min(FRAME - self.rw, len(tags) - i)
            self.ring[self.cur][self.rw:self.rw + n] = tags[i:i + n]
            self.rw += n
            i += n
        self.submitted += len(tags)


def run(fixed, scenario):
    s = Sim(fixed)
    preds = {}
    tag = 0
    if scenario.startswith("boot"):
        # The playback task starts (audio_output_start) some EOFs after the
        # enable in audio_output_init.
        s.advance(int(scenario[4:]) * FRAME + 100)
        s.restart_writer()
    for blk in range(400):
        if scenario == "stall" and blk == 150:
            s.advance(30000)                      # writer starved ~0.7 s
        if scenario == "restart" and blk == 150:
            s.advance(30000)                      # task stopped, DMA runs on
            s.restart_writer()
        preds[tag] = s.t + s.pipeline()
        s.write(list(range(tag, tag + BLOCK)))
        tag += BLOCK
    s.advance(DESC * FRAME * 2)
    errs = [s.played[k] - p for k, p in preds.items() if k in s.played]
    late = errs[-200:]                            # steady state after the event
    return min(late), max(late), errs[:3]


def main():
    ok = True
    for scen in ("enable", "boot0", "boot2", "boot5", "boot40", "stall",
                 "restart"):
        for fixed in (False, True):
            lo, hi, first = run(fixed, scen)
            if fixed:   # the first blocks too, not just the steady state
                lo, hi = min(lo, *first), max(hi, *first)
            ms = lambda f: f / 44.1
            tag = "new" if fixed else "old"
            print(f"{scen:8s} {tag}: true - predicted = {ms(lo):6.2f} .. {ms(hi):6.2f} ms"
                  f"  (first blocks {', '.join(f'{ms(e):.1f}' for e in first)})")
            if fixed and (lo < -2 or hi > 2):     # within 2 frames (45 us)
                ok = False
            if not fixed and scen == "boot2" and hi < (DESC - 3) * FRAME - 2:
                ok = False                         # the model must show the bug
    print("all passed" if ok else "FAILED")
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
