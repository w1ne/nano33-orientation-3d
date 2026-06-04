#!/usr/bin/env python3
"""Record firmware orientation samples from the bridge WebSocket for analysis.
Captures q + gyroMag with timestamps, then reports whether yaw keeps changing
*after* the board's rotation rate has dropped (the post-move-transient signature)."""
import asyncio, json, math, time, sys
import websockets

DUR = float(sys.argv[1]) if len(sys.argv) > 1 else 20.0

def yaw(q): return math.degrees(math.atan2(2*(q[0]*q[3]+q[1]*q[2]), 1-2*(q[2]*q[2]+q[3]*q[3])))

async def main():
    samples = []
    async with websockets.connect("ws://localhost:8765") as ws:
        t0 = time.time()
        while time.time() - t0 < DUR:
            try:
                msg = await asyncio.wait_for(ws.recv(), timeout=DUR)
            except asyncio.TimeoutError:
                break
            d = json.loads(msg)
            samples.append((time.time() - t0, yaw(d["q"]), d["gyroMag"]))
    # analysis
    if len(samples) < 20:
        print("too few samples:", len(samples)); return
    print(f"captured {len(samples)} samples over {samples[-1][0]:.1f}s")
    print(f"peak rate {max(s[2] for s in samples):.1f} deg/s (the fast move)")

    # Auto-find the LONGEST still segment (rate < 2 deg/s) and measure yaw drift there.
    best=(0,0,0); cur_s=None
    for i,(t,y,r) in enumerate(samples):
        if r < 2.0:
            if cur_s is None: cur_s=i
            dur=samples[i][0]-samples[cur_s][0]
            if dur>best[0]: best=(dur,cur_s,i)
        else:
            cur_s=None
    dur,a,b=best
    if dur>=2.0:
        seg=samples[a:b+1]; ys=[s[1] for s in seg]; rng=max(ys)-min(ys)
        drift=(seg[-1][1]-seg[0][1])/dur*60
        print(f">>> LONGEST STILL SEGMENT: {dur:.1f}s")
        print(f">>> yaw range in that still segment: {rng:.2f} deg")
        print(f">>> yaw drift rate while still:     {drift:+.1f} deg/min")
        print(f"    -> {'LOCKED (good)' if rng<5 else 'STILL DRIFTING'}")
    else:
        print(">>> no still segment >=2s found (board moved the whole time)")
    return
    # after the move: track when rate falls below 1 deg/s ("physically stopped"),
    # then measure how much yaw STILL changes after that point.
    # Always report the final-stillness settle window (robust to move timing).
    end_t = samples[-1][0]
    for win in (8.0, 5.0, 2.0):
        w = [y for (t, y, r) in samples if end_t - t <= win]
        rr = [r for (t, y, r) in samples if end_t - t <= win]
        if w:
            print(f">>> FINAL {int(win)}s: yaw range={max(w)-min(w):6.2f} deg, "
                  f"max rate={max(rr):.2f} deg/s  (small yaw range + low rate = LOCKED)")

    stop_i = None
    for i in range(peak_i, len(samples)):
        if samples[i][2] < 1.0:
            stop_i = i; break
    if stop_i is None:
        print("(peak was at very end; rely on FINAL-window numbers above)"); return
    st, sy, sr = samples[stop_i]
    final_y = samples[-1][1]
    print(f"rate dropped below 1 deg/s at t={st:.2f}s, yaw there = {sy:.1f} deg")
    print(f"yaw at end of capture        t={samples[-1][0]:.2f}s, yaw     = {final_y:.1f} deg")
    print(f">>> yaw CONTINUED to change by {final_y - sy:+.1f} deg AFTER the board's rate settled")
    # also report residual rate in the 0.5s right after stop
    tail = [r for (t, y, r) in samples[stop_i:] if t - st < 1.0]
    if tail:
        print(f">>> residual rate in 1s after stop: avg={sum(tail)/len(tail):.2f} max={max(tail):.2f} deg/s")
    # Does yaw SETTLE? Look at the final 5s and final 2s (board should be held still).
    end_t = samples[-1][0]
    last5 = [y for (t, y, r) in samples if end_t - t <= 5.0]
    last2 = [y for (t, y, r) in samples if end_t - t <= 2.0]
    if last5:
        print(f">>> yaw range in final 5s: {max(last5)-min(last5):.2f} deg   "
              f"final 2s: {max(last2)-min(last2):.2f} deg   (small = LOCKED/settled)")

asyncio.run(main())
