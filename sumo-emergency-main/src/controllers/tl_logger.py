#!/usr/bin/env python3
import csv, os
from typing import Dict, Any

class TLLogger:
    def __init__(self, path: str = "data/tls_logs.csv"):
        self.path = path
        os.makedirs(os.path.dirname(path), exist_ok=True)
        self._header_written = os.path.exists(path) and os.path.getsize(path) > 0

    def log(self, feat: Dict[str, Any], decision: Dict[str, Any]):
        row = {**feat,
               "extend_seconds": float(decision.get("extend", 0.0)),
               "switch_flag": int(bool(decision.get("force_switch", False)))}
        with open(self.path, "a", newline="", encoding="utf-8") as f:
            w = csv.DictWriter(f, fieldnames=list(row.keys()))
            if not self._header_written:
                w.writeheader()
                self._header_written = True
            w.writerow(row)
