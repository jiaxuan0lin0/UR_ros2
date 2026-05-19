import json
import os
import tempfile
import time
from pathlib import Path


class StateWriter:
    def __init__(self, path: Path, rate_hz: float):
        self.path = Path(path)
        self.period_sec = 1.0 / max(float(rate_hz), 1.0)
        self._next_write_time = 0.0
        self.path.parent.mkdir(parents=True, exist_ok=True)

    def should_write(self) -> bool:
        return time.monotonic() >= self._next_write_time

    def maybe_write(self, state: dict, force: bool = False):
        now = time.monotonic()
        if not force and now < self._next_write_time:
            return
        self._next_write_time = now + self.period_sec

        payload = dict(state)
        payload["wall_time"] = time.time()
        fd, tmp_name = tempfile.mkstemp(
            prefix=f".{self.path.name}.",
            suffix=".tmp",
            dir=str(self.path.parent),
            text=True,
        )
        try:
            with os.fdopen(fd, "w") as tmp:
                json.dump(payload, tmp)
            os.replace(tmp_name, self.path)
        finally:
            if os.path.exists(tmp_name):
                os.unlink(tmp_name)
