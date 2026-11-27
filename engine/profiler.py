# 물리 스텝 시간 측정 도구

import time
from contextlib import contextmanager

class Profiler:
    def __init__(self):
        self.history = [] # 스텝마다 기록
        self.last = {} # 마지막 스텝 기록
        self._cur = None # 지금 기록 중인 스텝
        self._t0 = 0.0

    def begin_step(self):
        self._cur = {}
        self._t0 = time.perf_counter()

    def end_step(self):
        self._cur["step_ms"] = (time.perf_counter() - self._t0) * 1000.0
        self.last = self._cur
        self.history.append(self._cur)
        self._cur = None

    @contextmanager
    def section(self, name):
        t = time.perf_counter()
        try:
            yield
        finally:
            if self._cur is not None:
                key = name + "_ms"
                self._cur[key] = self._cur.get(key, 0.0) + (time.perf_counter() - t) * 1000.0

    def set(self, name, value):
        if self._cur is not None:
            self._cur[name] = value

    def mean(self, key, last_n=None):
        rows = self.history[-last_n:] if last_n else self.history
        values = [r.get(key, 0.0) for r in rows]
        return sum(values) / len(values) if values else 0.0