import json, os, time
from typing import Any, Dict
from config import LOG_DIR

class RunLogger:
    def __init__(self):
        ts = time.strftime("%Y%m%d_%H%M%S")
        os.makedirs(LOG_DIR, exist_ok=True)
        self.path = os.path.join(LOG_DIR, f"run_{ts}.json")
        self.buffer: Dict[str, Any] = {"events": [], "meta": {"start_time": ts}}

    def event(self, **kwargs):
        kwargs["t"] = time.time()
        self.buffer["events"].append(kwargs)

    def flush(self):
        with open(self.path, "w", encoding="utf-8") as f:
            json.dump(self.buffer, f, indent=2)

    def close(self):
        self.flush()