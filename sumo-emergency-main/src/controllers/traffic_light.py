#!/usr/bin/env python3
"""
Trafik Işığı Denetleyici (kural tabanlı ANFIS-gibi + opsiyonel ANFIS)
"""
from typing import Optional, Dict, List
from dataclasses import dataclass
from src.ai.anfis import RuleBasedTLS, RuleConfig
from src.controllers.tl_logger import TLLogger

@dataclass
class TLSMap:
    tls_id: str
    main_green_phases: List[int]
    side_green_phases: List[int]
    lanes_main: List[str]
    lanes_side: List[str]
    min_green: int = 5
    max_green: int = 40
    extend_clip: tuple = (0.0, 10.0)

class TrafficLightController:
    def __init__(self, main_junction_id: Optional[str] = None, tls_map: Optional[TLSMap] = None):
        self.main_junction_id = main_junction_id
        self.normal_programs: Dict[str, str] = {}
        self.tls_map: Optional[TLSMap] = tls_map

        cfg = RuleConfig(
            min_green=tls_map.min_green if tls_map else 5,
            max_green=tls_map.max_green if tls_map else 40,
            extend_clip=tls_map.extend_clip if tls_map else (0.0, 10.0),
        )
        self._rule = RuleBasedTLS(cfg)
        self._green_timer: int = 0
        self._prev_phase: Optional[int] = None

        self._logger = TLLogger("data/tls_logs.csv")
        self._anfis_runtime = None  # lazy load (enable_anfis ile)

    # İsteğe bağlı: eğitimli ANFIS'i etkinleştir
    def enable_anfis(self, model_path: str):
        from src.ai.anfis_runtime import ANFISRuntime
        self._anfis_runtime = ANFISRuntime(model_path, extend_clip=self.tls_map.extend_clip if self.tls_map else (0.0,10.0))

    # --- emergency override (mevcut fonksiyonların birebir korunması) ---
    def give_priority(self, junction_id: str, approach_edge_id: str, green_seconds: float = 20.0) -> bool:
        try:
            import traci
            if junction_id not in self.normal_programs:
                self.normal_programs[junction_id] = str(traci.trafficlight.getProgram(junction_id))

            # Mevcut faz uzunluğuna göre state uzunluğunu garanti etmek için controlled links'i al
            links = traci.trafficlight.getControlledLinks(junction_id)

            # Başlangıç hepsi kırmızı
            state = ['r'] * len(links)

            # İlgili yaklaşımın tüm çıkış linklerine KORUMALI yeşil ver (g)
            for idx, group in enumerate(links):
                for in_lane, _, _ in group:
                    if approach_edge_id and in_lane.startswith(approach_edge_id + "_"):
                        state[idx] = 'g'

            state_green = ''.join(state)  # örn: rrgrr...

            # Kısa bir sarı/temizleme fazı ekleyelim (isteğe bağlı ama iyi pratik)
            state_yellow = ''.join('y' if c == 'g' else 'r' for c in state)

            logic = traci.trafficlight.Logic(
                programID="emergency",
                type=0,  # static
                currentPhaseIndex=0,
                phases=[
                    traci.trafficlight.Phase(int(green_seconds), state_green),
                    traci.trafficlight.Phase(3, state_yellow),  # temizleme
                ],
            )
            traci.trafficlight.setProgramLogic(junction_id, logic)
            traci.trafficlight.setProgram(junction_id, "emergency")
            traci.trafficlight.setPhase(junction_id, 0)
            return True
        except Exception:
            return False

    def restore(self, junction_id: str) -> bool:
        try:
            import traci
            prog = self.normal_programs.get(junction_id)
            if prog is None:
                return True
            traci.trafficlight.setProgram(junction_id, prog)
            return True
        except Exception:
            return False

    # --- metrik yardımcıları ---
    @staticmethod
    def _queue(lanes: List[str]) -> int:
        import traci
        return int(sum(traci.lane.getLastStepHaltingNumber(l) for l in lanes))

    @staticmethod
    def _wait_avg(lanes: List[str]) -> float:
        import traci
        total_w, total_n = 0.0, 0
        for l in lanes:
            total_w += traci.lane.getWaitingTime(l)
            total_n += max(1, traci.lane.getLastStepVehicleNumber(l))
        return total_w / max(1, total_n)

    def run_step(self) -> None:
        if not self.tls_map:
            return
        import traci
        tid = self.tls_map.tls_id
        phase = traci.trafficlight.getPhase(tid)
        on_main = phase in self.tls_map.main_green_phases

        if self._prev_phase != phase:
            self._prev_phase = phase
            self._green_timer = 0
        else:
            self._green_timer += 1

        qm = self._queue(self.tls_map.lanes_main)
        qs = self._queue(self.tls_map.lanes_side)
        wm = self._wait_avg(self.tls_map.lanes_main)
        ws = self._wait_avg(self.tls_map.lanes_side)

        features = {
            "queue_main": qm, "queue_side": qs,
            "wait_main": wm, "wait_side": ws,
            "on_main_green": int(on_main),
            "ambu_main": 0, "ambu_side": 0,
        }

        # Kararı ver: ANFIS aktifse onu kullan, yoksa kural tabanlı
        if self._anfis_runtime is not None:
            decision = self._anfis_runtime.decide(features, self.tls_map.min_green, self._green_timer)
        else:
            decision = self._rule.decide(features, {"on_main_green": on_main, "green_timer": self._green_timer})

        # Logla (eğitim verisi)
        log_feat = dict(features)
        log_feat["green_timer"] = self._green_timer
        self._logger.log(log_feat, decision)

        # Uygula
        if decision.get("force_switch") and self._green_timer >= self.tls_map.min_green:
            next_phase = (self.tls_map.side_green_phases[0] if on_main
                          else self.tls_map.main_green_phases[0])
            traci.trafficlight.setPhase(tid, next_phase)
            self._green_timer = 0
        else:
            ext = float(decision.get("extend", 0.0))
            if ext > 0.0 and self._green_timer <= self.tls_map.max_green:
                remaining = traci.trafficlight.getNextSwitch(tid) - traci.simulation.getTime()
                traci.trafficlight.setPhaseDuration(tid, max(0.0, remaining) + ext)
