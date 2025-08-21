#!/usr/bin/env python3
"""
ANFIS Yer Tutucu + Kural Tabanlı TLS
"""

from dataclasses import dataclass
from typing import Dict, Tuple

class ANFIS:
    def __init__(self):
        self.max_increase_ratio = 0.25  # h(n) en fazla %25 artırılsın
        self.base_signal_delay = 2.0    # s

    def adjust_heuristic(self, base_h: float, context: Dict) -> float:
        increase = min(self.max_increase_ratio, 0.15)
        return float(base_h * (1.0 + increase))

    def signal_delay(self, node_id: str, context: Dict) -> float:
        return float(self.base_signal_delay)

# --- Kural tabanlı “ANFIS-gibi” TLS karar verici ----------------------------

@dataclass
class RuleConfig:
    min_green: int = 5
    max_green: int = 40
    extend_clip: Tuple[float, float] = (0.0, 10.0)

class RuleBasedTLS:
    """
    Girdi: queue_main/side, wait_main/side, ambu_main/side
    Bağlam: on_main_green, green_timer
    Çıktı: {'extend': float, 'force_switch': bool}
    """
    def __init__(self, cfg: RuleConfig):
        self.cfg = cfg

    def _clip(self, x: float) -> float:
        lo, hi = self.cfg.extend_clip
        return max(lo, min(hi, x))

    def decide(self, features: Dict, context: Dict) -> Dict[str, float | bool]:
        on_main = bool(context.get("on_main_green", True))
        t_green = int(context.get("green_timer", 0))

        qm = float(features.get("queue_main", 0))
        qs = float(features.get("queue_side", 0))
        wm = float(features.get("wait_main", 0))
        ws = float(features.get("wait_side", 0))
        ambm = int(features.get("ambu_main", 0))
        ambs = int(features.get("ambu_side", 0))

        # 1) Ambulans önceliği
        if on_main and ambm:
            return {"extend": self._clip(9.0), "force_switch": False}
        if (not on_main) and ambs:
            return {"extend": self._clip(9.0), "force_switch": False}
        if on_main and ambs and t_green >= self.cfg.min_green:
            return {"extend": 0.0, "force_switch": True}
        if (not on_main) and ambm and t_green >= self.cfg.min_green:
            return {"extend": 0.0, "force_switch": True}

        # 2) Normal durum
        main_score = qm + (wm / 10.0)
        side_score = qs + (ws / 10.0)
        diff = main_score - side_score

        base = 2.0 if t_green < self.cfg.min_green else 0.5

        if on_main:
            if diff >= 3.0:
                ext = base + 4.0
            elif diff >= 1.2:
                ext = base + 2.0
            elif diff <= -2.0:
                ext = 0.0
            else:
                ext = base + 1.0
        else:
            if diff <= -3.0:
                ext = base + 4.0
            elif diff <= -1.2:
                ext = base + 2.0
            elif diff >= 2.0:
                ext = 0.0
            else:
                ext = base + 1.0

        return {"extend": self._clip(ext), "force_switch": False}
