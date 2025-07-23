import time
from collections import deque
from datetime import datetime


class SignFlipDetector:
    def __init__(self, filename: str,
                 poll_interval: float = 0.1,
                 stable_window: float = 1.0,
                 zupt_window: float = 0.4,
                 zupt_thresh: float = 0.3) -> None:
        self.filename = filename
        self.poll_interval = poll_interval
        self.stable_window = stable_window

        self.recent_v = deque(maxlen=3)
        self.seen_line_count = 0

        self.prev_ts = None
        self.prev_acc = None
        self.vel_z = 0.0

        self.zupt_window = zupt_window
        self.zupt_thresh = zupt_thresh
        self.zupt_buf = deque(maxlen=int(zupt_window / poll_interval))

        self.window_start_time = None
        self.window_start_timestamp = None
        self.window_stable = True
        self.expected_sign = None
        self.message_printed = False
        self.violation_time = None
        self.finished = False
        self.current_ts_log = None
        self.last_nz_v = None
        self.last_flip_time = None
        self.prelim_flip_time = None
        self.prelim_flip_ts = None

        self.vel_log_path = "velocity_output.txt"
        self.vel_log_file = open(self.vel_log_path, "w", encoding="utf-8")

    def _update_recent(self, v: float) -> None:
        self.recent_v.append(v)

    def _sign_change_detected(self, v_now: float) -> bool:
        if v_now == 0:
            return False
        changed = (self.last_nz_v is not None and self.last_nz_v * v_now < 0)
        self.last_nz_v = v_now
        return changed

    def _print_once(self, msg: str) -> None:
        if not self.message_printed:
            print(msg)
            self.message_printed = True

    def _read_new_lines(self):
        with open(self.filename, "r", encoding="utf-8") as f:
            lines = f.readlines()
        new_lines = lines[self.seen_line_count:]
        self.seen_line_count = len(lines)
        return new_lines

    def _parse_line(self, line: str):
        if line.startswith("Timestamp :"):
            ts_str = line.split("Timestamp :")[1].strip()
            try:
                return "ts", datetime.fromisoformat(ts_str)
            except ValueError:
                return "ts", None
        elif line.startswith("Accel"):
            try:
                raw = line.split("[")[1].split("]")[0]
                acc_z = float(raw.split(",")[2].strip())
                return "acc", acc_z
            except Exception as e:
                print(f"⚠️ 파싱 오류 (accel): {e}")
                return None, None
        return None, None

    def _update_velocity(self, acc_z: float):
        now = time.monotonic()
        if self.prev_ts is not None:
            dt = now - self.prev_ts
            self.vel_z += 0.5 * (self.prev_acc + acc_z) * dt
            self._apply_zupt()
            self._log_velocity()
        self.prev_ts = now
        self.prev_acc = acc_z

        if not self.finished:
            self._update_recent(self.vel_z)
            if self.vel_z != 0:
                if self.last_nz_v is not None and self.last_nz_v * self.vel_z < 0:
                    if self.window_start_time is None:
                        self.window_start_time = now
                        self.window_start_timestamp = self.current_ts_log or "—"
                        self.window_stable = True
                        self.expected_sign = -1 if self.vel_z < 0 else 1
                        self.message_printed = False
                        self.violation_time = None
                self.last_nz_v = self.vel_z
            self._handle_sign_flip(now)

    def _apply_zupt(self):
        self.zupt_buf.append(abs(self.vel_z))
        if len(self.zupt_buf) == self.zupt_buf.maxlen and max(self.zupt_buf) <= self.zupt_thresh:
            self.vel_z = 0.0

    def _log_velocity(self):
        if self.current_ts_log:
            self.vel_log_file.write(f"{self.current_ts_log.isoformat()}, {self.vel_z:.4f}\n")
            self.vel_log_file.flush()

    def _handle_sign_flip(self, now_mono: float):
        if self._sign_change_detected(self.vel_z) and self.window_start_time is None:
            self.window_start_time = now_mono
            self.window_start_timestamp = self.current_ts_log or "—"
            self.window_stable = True
            self.expected_sign = -1 if self.vel_z < 0 else 1
            self.message_printed = False
            self.violation_time = None
            self.prelim_flip_time = now_mono
            self.prelim_flip_ts = self.current_ts_log

        if self.window_start_time is not None:
            elapsed = now_mono - self.window_start_time
            self._print_once(
                f"📍 부호 변화! 시간: {self.window_start_timestamp} "
                f"(v = {self.vel_z:.2f} m/s)"
            )

            if (self.expected_sign == -1 and self.vel_z > 0) or \
               (self.expected_sign == 1 and self.vel_z < 0):
                self.window_stable = False
                self.violation_time = self.violation_time or now_mono

            if elapsed >= self.stable_window:
                if self.window_stable:
                    print(f"✅ True – {self.window_start_timestamp} 이후 "
                          f"{self.stable_window:.1f}s 동안 부호 유지 "
                          f"(실제 {elapsed:.2f}s)")
                    print(f"최고점 도달 시간: {self.window_start_timestamp}")
                    self.last_flip_time = self.current_ts_log
                    self.finished = True

                else:
                    held = ((self.violation_time - self.window_start_time)
                            if self.violation_time else elapsed)
                    print(f"❌ 부호 유지 실패 – 유지 {held:.2f}s")

                self.window_start_time = None
                self.window_start_timestamp = None
                self.expected_sign = None
                self.window_stable = True
                self.message_printed = False
                self.violation_time = None

    def close(self):
        self.vel_log_file.close()


class ChuteDeltaAccelDetector:
    def __init__(self, delta_thresh: float = 5.0, window: float = 2.0):
        self.prev_acc = None
        self.delta_thresh = delta_thresh
        self.window = window
        self.buffer = deque(maxlen=int(window * 30))
        self.pending_triggers = []

        self.triggered = False
        self.fail_safe_triggered = False
        self.printed_triggers = set()

        self.flip_confirmed_time = None

    def update_flip_confirmed(self, confirmed_time: float):
        self.flip_confirmed_time = confirmed_time
        self._process_pending_triggers()

    def _process_pending_triggers(self):
        for a, prev, ts, t in list(self.pending_triggers):
            if self.flip_confirmed_time - self.window <= t <= self.flip_confirmed_time + self.window and t not in self.printed_triggers:
                delta_t = t - self.flip_confirmed_time
                print(f"🪂 낙하산 감지! 시간: {ts}")
                print(f"    ∆acc_z = {a - prev:.2f} m/s² (최고점 확정 후 +{delta_t:.2f}s)")
                self.triggered = True
                self.printed_triggers.add(t)

        self.pending_triggers = [
            (a, prev, ts, t) for (a, prev, ts, t) in self.pending_triggers
            if t not in self.printed_triggers
        ]

    def process(self, acc_z, timestamp):
        now = time.monotonic()
        self.buffer.append((acc_z, timestamp, now))

        if self.prev_acc is not None and abs(acc_z - self.prev_acc) >= self.delta_thresh:
            self.pending_triggers.append((acc_z, self.prev_acc, timestamp, now))

        if self.flip_confirmed_time is not None and not self.triggered:
            for a, prev, ts, t in list(self.pending_triggers):
                if self.flip_confirmed_time - self.window <= t <= self.flip_confirmed_time + self.window and t not in self.printed_triggers:
                    delta_t = t - self.flip_confirmed_time
                    print(f"🪂 낙하산 사출 감지! 시간: {ts}, ∆acc_z = {a - prev:.2f} m/s²acc_z = {a - prev:.2f} m/s\xb2 "
                          f"(최고점 확정 후 +{delta_t:.2f}s)")
                    self.triggered = True
                    self.printed_triggers.add(t)

            self.pending_triggers = [
                (a, prev, ts, t) for (a, prev, ts, t) in self.pending_triggers
                if t not in self.printed_triggers
            ]

        if self.flip_confirmed_time is not None and not self.triggered and not self.fail_safe_triggered:
            if now - self.flip_confirmed_time > self.window:
                print(f"❗Fail-safe mechanism 가능! (최고점 후 {self.window:.1f}초 내 낙하산 미감지)")
                self.fail_safe_triggered = True

        self.prev_acc = acc_z


if __name__ == "__main__":
    detector = SignFlipDetector("filtered_test_data.txt",
                                poll_interval=0.1,
                                stable_window=1.0)
    chute_detector = ChuteDeltaAccelDetector(delta_thresh=5.0)

    print("📱 accel_z 적분 기반 속도 추정 + 부호 변화 감시")
    try:
        while True:
            new_lines = detector._read_new_lines()
            for line in new_lines:
                key, value = detector._parse_line(line.strip())
                if key == "ts":
                    detector.current_ts_log = value
                elif key == "acc":
                    detector._update_velocity(value)
                    chute_detector.process(value, detector.current_ts_log)

                    if detector.finished and chute_detector.flip_confirmed_time is None:
                        chute_detector.update_flip_confirmed(time.monotonic())

            time.sleep(detector.poll_interval)
    except KeyboardInterrupt:
        print("\n🛑 감시 수동 중단")
    finally:
        detector.close()
