#!/usr/bin/env python3
"""
Trafik Işığı Denetleyici (ANFIS tabanlı)

Amaç: Ambulans yaklaşımı için fazı ayarlamak ve yeşili korumak. XGBoost kaldırıldı;
tetikleme ve yeşil uzatmayı ANFIS çıkarımı ile yapıyoruz.
"""

from typing import Optional, Dict, Any, Tuple
import os
import logging

from src.ai.anfis import AnfisModel

logger = logging.getLogger(__name__)


class TrafficLightController:
	def __init__(self, main_junction_id: Optional[str] = None, anfis_model_path: Optional[str] = None):
		self.main_junction_id = main_junction_id
		self.normal_programs: Dict[str, str] = {}
		self.last_actions: Dict[str, Tuple[float, str]] = {}
		self.last_state_applied: Dict[str, str] = {}
		self.active_priority: Dict[str, Dict[str, Any]] = {}
		try:
			model_file = anfis_model_path or os.environ.get("ANFIS_MODEL", "models/anfis.json")
			self.anfis_model = AnfisModel(model_file if os.path.exists(model_file) else None)
			if self.anfis_model and self.anfis_model.loaded:
				logger.info(f"ANFIS model yüklendi: {model_file}")
			else:
				logger.info("ANFIS varsayılan kural tabanı ile çalışıyor (model dosyası bulunamadı)")
		except Exception as e:
			logger.warning(f"ANFIS init hatası: {e}")
			self.anfis_model = AnfisModel(None)

	def _list_approach_edges(self, junction_id: str) -> Dict[int, str]:
		try:
			import traci
			links = traci.trafficlight.getControlledLinks(junction_id)
			edges: Dict[int, str] = {}
			for idx, group in enumerate(links):
				for in_lane, _out_lane, _via in group:
					if "_" in in_lane:
						edges[idx] = in_lane.split("_")[0]
						break
			return edges
		except Exception:
			return {}

	def _estimate_eta(self, vehicle_id: str, junction_id: str) -> Tuple[float, float]:
		try:
			import traci, math
			try:
				next_tls = traci.vehicle.getNextTLS(vehicle_id)
				for tls_id, _idx, dist, _state in next_tls:
					if str(tls_id) == str(junction_id):
						v = max(1.0, float(traci.vehicle.getSpeed(vehicle_id)))
						return float(dist), float(dist) / v
			except Exception:
				pass
			vx, vy = traci.vehicle.getPosition(vehicle_id)
			jx, jy = traci.junction.getPosition(junction_id)
			d = math.hypot(vx - jx, vy - jy)
			v = max(1.0, float(traci.vehicle.getSpeed(vehicle_id)))
			return float(d), float(d) / v
		except Exception:
			return 9999.0, 9999.0

	def _approach_angle_cos(self, vehicle_id: str, junction_id: str) -> float:
		try:
			import traci, math
			vx, vy = traci.vehicle.getPosition(vehicle_id)
			jx, jy = traci.junction.getPosition(junction_id)
			dx, dy = (jx - vx), (jy - vy)
			len_v = math.hypot(dx, dy)
			if len_v <= 1e-3:
				return 1.0
			ux, uy = dx / len_v, dy / len_v
			angle_deg = float(traci.vehicle.getAngle(vehicle_id))
			rad = math.radians(angle_deg)
			vxh, vyh = math.cos(rad), math.sin(rad)
			return float(max(-1.0, min(1.0, ux * vxh + uy * vyh)))
		except Exception:
			return 0.0

	def _extract_features_for_approach(self, junction_id: str, approach_edge_id: str, sim_time: float, vehicle_id: Optional[str]) -> Dict[str, float]:
		feats: Dict[str, float] = {}
		try:
			import traci
			phase_index = float(traci.trafficlight.getPhase(junction_id))
			veh_total = 0.0
			speed_sum = 0.0
			halt_total = 0.0
			lane_count = 0.0
			links = traci.trafficlight.getControlledLinks(junction_id)
			for _gidx, group in enumerate(links):
				for in_lane, _out_lane, _via in group:
					if approach_edge_id and in_lane.startswith(approach_edge_id + "_"):
						try:
							veh_total += float(traci.lane.getLastStepVehicleNumber(in_lane))
							speed_sum += float(traci.lane.getLastStepMeanSpeed(in_lane))
							halt_total += float(traci.lane.getLastStepHaltingNumber(in_lane))
							lane_count += 1.0
						except Exception:
							continue
			mean_speed = (speed_sum / lane_count) if lane_count > 0 else 0.0
			try:
				next_sw = float(traci.trafficlight.getNextSwitch(junction_id))
				phase_remaining = max(0.0, next_sw - float(sim_time))
			except Exception:
				phase_remaining = 0.0
			dist_to_j, eta_to_j = (0.0, 0.0)
			if vehicle_id is not None:
				dist_to_j, eta_to_j = self._estimate_eta(vehicle_id, junction_id)
			angle_cos = self._approach_angle_cos(vehicle_id, junction_id) if vehicle_id is not None else 0.0
			feats = {
				"phase_index": phase_index,
				"phase_remaining": phase_remaining,
				"veh_approach": veh_total,
				"halt_approach": halt_total,
				"v_approach": mean_speed,
				"dist_to_tls": float(dist_to_j),
				"eta_seconds": float(eta_to_j),
				"angle_cos": float(angle_cos),
			}
		except Exception:
			pass
		return feats

	def _log_dir_training_row(self, feats: Dict[str, float], label_select: int, y_extend: float, junction_id: str, approach_edge_id: str, sim_time: float) -> None:
		import os, csv
		log_path = os.path.join("data", "dir_training.csv")
		os.makedirs(os.path.dirname(log_path), exist_ok=True)
		row = {**feats, "label_select": int(label_select), "y_extend": float(y_extend), "junction_id": junction_id, "approach_edge_id": approach_edge_id, "t": float(sim_time)}
		write_header = not os.path.exists(log_path) or os.path.getsize(log_path) == 0
		with open(log_path, "a", newline="", encoding="utf-8") as f:
			w = csv.DictWriter(f, fieldnames=list(row.keys()))
			if write_header:
				w.writeheader()
			w.writerow(row)

	def _log_signal_training_row(self, feats: Dict[str, float], y_extend: float, junction_id: str, approach_edge_id: str, sim_time: float, action: str = "extend") -> None:
		import os, csv
		# v2: Yeni şema (ANFIS) — eski dosyayla karışmayı önlemek için ayrı dosya
		log_path = os.path.join("data", "signal_training_v2.csv")
		os.makedirs(os.path.dirname(log_path), exist_ok=True)
		row = {**feats, "y_extend": float(y_extend), "action": str(action), "junction_id": junction_id, "approach_edge_id": approach_edge_id, "t": float(sim_time)}
		write_header = not os.path.exists(log_path) or os.path.getsize(log_path) == 0
		with open(log_path, "a", newline="", encoding="utf-8") as f:
			w = csv.DictWriter(f, fieldnames=list(row.keys()))
			if write_header:
				w.writeheader()
			w.writerow(row)

	def _safe_apply(self, junction_id: str, state_str: str, green_seconds: float) -> bool:
		try:
			import traci
			prev_state = self.last_state_applied.get(junction_id)
			edges_by_idx = {}
			try:
				links_dbg = traci.trafficlight.getControlledLinks(junction_id)
				for idx, group in enumerate(links_dbg):
					for in_lane, _out_lane, _via in group:
						if "_" in in_lane:
							edges_by_idx[idx] = in_lane.split("_")[0]
							break
			except Exception:
				edges_by_idx = {}
			traci.trafficlight.setRedYellowGreenState(junction_id, state_str)
			try:
				traci.trafficlight.setPhaseDuration(junction_id, float(green_seconds))
			except Exception as e:
				logger.debug(f"[TL] _safe_apply: setPhaseDuration failed: {e}")
				try:
					current_phase = traci.trafficlight.getPhase(junction_id)
					traci.trafficlight.setPhase(junction_id, current_phase)
				except Exception:
					pass
			try:
				greens_on: list = []
				reds_on: list = []
				if isinstance(prev_state, str) and len(prev_state) == len(state_str):
					for i, ch in enumerate(state_str):
						prev_ch = prev_state[i]
						edge_id = edges_by_idx.get(i, f"idx:{i}")
						if (ch in ('G','g')) and (prev_ch not in ('G','g')):
							greens_on.append(str(edge_id))
						elif (ch in ('r','R')) and (prev_ch not in ('r','R')):
							reds_on.append(str(edge_id))
				if greens_on or reds_on:
					logger.info(f"[TL] STATE CHANGE tl={junction_id} greens_on={len(greens_on)} reds_on={len(reds_on)} state={state_str}")
					if greens_on:
						logger.info(f"[TL] GREEN_ON tl={junction_id}: " + ", ".join(greens_on))
					if reds_on:
						logger.info(f"[TL] RED_ON tl={junction_id}: " + ", ".join(reds_on))
			except Exception:
				pass
			self.last_state_applied[junction_id] = state_str
			return True
		except Exception as e:
			logger.debug(f"[TL] _safe_apply error: {e}")
			return False

	def set_ambulance_priority(self, traffic_light_id: str, approach_edge_id: str, green_seconds: float = 20.0, ambulance_id: str = None) -> bool:
		try:
			import traci
			if traffic_light_id not in self.normal_programs:
				self.normal_programs[traffic_light_id] = str(traci.trafficlight.getProgram(traffic_light_id))
			ambulance_lane = None
			if ambulance_id:
				try:
					ambulance_lane = traci.vehicle.getLaneID(ambulance_id)
				except Exception:
					pass
			state = list(traci.trafficlight.getRedYellowGreenState(traffic_light_id))
			links = traci.trafficlight.getControlledLinks(traffic_light_id)
			for idx, group in enumerate(links):
				is_ambulance_direction = False
				for in_lane, _out_lane, _ in group:
					if ambulance_lane and in_lane == ambulance_lane:
						is_ambulance_direction = True
						break
					elif approach_edge_id and in_lane.startswith(approach_edge_id + "_"):
						is_ambulance_direction = True
						break
					elif approach_edge_id and in_lane.split('#')[0] == approach_edge_id.split('#')[0]:
						is_ambulance_direction = True
						break
				state[idx] = 'G' if is_ambulance_direction else 'r'
			state_str = ''.join(ch if ch in 'GgYyRr' else 'r' for ch in state)
			dist_to_tls = float('inf')
			if ambulance_id:
				try:
					next_tls = traci.vehicle.getNextTLS(ambulance_id)
					if next_tls:
						dist_to_tls = float(next_tls[0][2])
				except Exception:
					pass
			green_count = state_str.count('G') + state_str.count('g')
			red_count = state_str.count('r') + state_str.count('R')
			prev_state = self.last_state_applied.get(traffic_light_id)
			if prev_state != state_str:
				logger.info(f"[TL] TRAFİK IŞIĞI DEĞİŞİMİ: tl={traffic_light_id} mesafe={dist_to_tls:.1f}m | Yeşil: {green_count} yön, Kırmızı: {red_count} yön | State: {state_str}")
			# ANFIS tahminli yeşil süresi
			try:
				if self.anfis_model is not None:
					feats_for_extend = {
						"dist_to_tls": dist_to_tls,
						"ambulance_speed": float(traci.vehicle.getSpeed(ambulance_id)) if ambulance_id else 0.0,
						"queue_length": 0.0,
						"eta_seconds": (dist_to_tls / max(0.5, float(traci.vehicle.getSpeed(ambulance_id)))) if ambulance_id else 9999.0,
						"phase_index": float(traci.trafficlight.getPhase(traffic_light_id)),
						"phase_remaining": max(0.0, float(traci.trafficlight.getNextSwitch(traffic_light_id)) - float(traci.simulation.getTime()))
					}
					try:
						links_loc = traci.trafficlight.getControlledLinks(traffic_light_id)
						qsum = 0.0
						for group in links_loc:
							for in_lane, _out_lane, _via in group:
								if approach_edge_id and in_lane.startswith(approach_edge_id + "_"):
									qsum += float(traci.lane.getLastStepVehicleNumber(in_lane)) * 7.5
						feats_for_extend["queue_length"] = qsum
					except Exception:
						pass
					green_seconds = float(self.anfis_model.predict_extend_seconds(feats_for_extend))
			except Exception:
				pass
			ok = self._safe_apply(traffic_light_id, state_str, green_seconds)
			if ok:
				self.active_priority[traffic_light_id] = {"ambulance_id": ambulance_id, "state": state_str}
				# Eğitim verisi: uygulanan yeşil süresi ve o anki özellikler
				try:
					feats_for_log = {
						"dist_to_tls": dist_to_tls,
						"ambulance_speed": float(traci.vehicle.getSpeed(ambulance_id)) if ambulance_id else 0.0,
						"queue_length": 0.0,
						"eta_seconds": (dist_to_tls / max(0.5, float(traci.vehicle.getSpeed(ambulance_id)))) if ambulance_id else 9999.0,
						"phase_index": float(traci.trafficlight.getPhase(traffic_light_id)),
						"phase_remaining": max(0.0, float(traci.trafficlight.getNextSwitch(traffic_light_id)) - float(sim_time := traci.simulation.getTime() if hasattr(traci, 'simulation') else 0.0))
					}
					# Kuyruk uzunluğu: yaklaşan ve ambulans şeridi
					try:
						links_loc = traci.trafficlight.getControlledLinks(traffic_light_id)
						qsum = 0.0
						for group in links_loc:
							for in_lane, _out_lane, _via in group:
								if approach_edge_id and in_lane.startswith(approach_edge_id + "_"):
									qsum += float(traci.lane.getLastStepVehicleNumber(in_lane)) * 7.5
						feats_for_log["queue_length"] = qsum
					except Exception:
						pass
					self._log_signal_training_row(feats_for_log, green_seconds, traffic_light_id, approach_edge_id, float(sim_time) if 'sim_time' in locals() else 0.0, action="extend")
				except Exception:
					pass
			return ok
		except Exception as e:
			logger.debug(f"[TL] set_ambulance_priority error: {e}")
			return False

	def maintain_active_priorities(self, release_distance_m: float = 50.0, keep_green_seconds: float = 1.5) -> None:
		try:
			import traci, math
			to_restore = []
			for tl_id, info in list(self.active_priority.items()):
				amb_id = str(info.get("ambulance_id") or "")
				state_str = str(info.get("state") or "")
				try:
					veh_ids = set(traci.vehicle.getIDList())
					if not amb_id or amb_id not in veh_ids:
						to_restore.append(tl_id)
						continue
				except Exception:
					to_restore.append(tl_id)
					continue
				try:
					vx, vy = traci.vehicle.getPosition(amb_id)
					jx, jy = traci.junction.getPosition(tl_id)
					d = math.hypot(vx - jx, vy - jy)
					is_upcoming = False
					try:
						next_tls = traci.vehicle.getNextTLS(amb_id)
						if next_tls:
							is_upcoming = str(next_tls[0][0]) == str(tl_id)
					except Exception:
						is_upcoming = False
				except Exception:
					to_restore.append(tl_id)
					continue
				if is_upcoming or (d <= float(release_distance_m)):
					if state_str:
						self._safe_apply(tl_id, state_str, float(keep_green_seconds))
				else:
					to_restore.append(tl_id)
			for tl_id in to_restore:
				if self.restore(tl_id):
					logger.info(f"[TL] Öncelik sonlandırıldı ve normale döndü: tl={tl_id}")
				self.active_priority.pop(tl_id, None)
		except Exception as e:
			logger.debug(f"[TL] maintain_active_priorities error: {e}")

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

	def _make_approach_green_state(self, junction_id: str, approach_edge_id: str) -> str:
		try:
			import traci
			state = list(traci.trafficlight.getRedYellowGreenState(junction_id))
			links = traci.trafficlight.getControlledLinks(junction_id)
			for idx, group in enumerate(links):
				for in_lane, _out_lane, _via in group:
					if approach_edge_id and in_lane.startswith(approach_edge_id + "_"):
						state[idx] = 'G'
			return ''.join(ch if ch in 'GgYyRr' else 'r' for ch in state)
		except Exception:
			return ""

	def should_trigger_priority(self, junction_id: str, approach_edge_id: str, sim_time: float, ambulance_id: str = None) -> bool:
		try:
			import traci
			if not ambulance_id:
				return False
			dist_to_tls = float('inf')
			v_ms = 0.0
			queue_len_m = 0.0
			try:
				next_tls = traci.vehicle.getNextTLS(ambulance_id)
				if next_tls:
					dist_to_tls = float(next_tls[0][2])
			except Exception:
				pass
			try:
				v_ms = float(traci.vehicle.getSpeed(ambulance_id))
			except Exception:
				pass
			try:
				ambulance_lane = None
				try:
					ambulance_lane = traci.vehicle.getLaneID(ambulance_id)
				except Exception:
					pass
				links_loc = traci.trafficlight.getControlledLinks(junction_id)
				for group in links_loc:
					for in_lane, _out_lane, _via in group:
						if approach_edge_id and in_lane.startswith(approach_edge_id + "_"):
							queue_len_m += float(traci.lane.getLastStepVehicleNumber(in_lane)) * 7.5
						elif ambulance_lane and in_lane == ambulance_lane:
							queue_len_m += float(traci.lane.getLastStepVehicleNumber(in_lane)) * 7.5
			except Exception:
				pass
			features = {
				"dist_to_tls": dist_to_tls,
				"ambulance_speed": v_ms,
				"queue_length": queue_len_m,
				"eta_seconds": (dist_to_tls / max(0.5, v_ms)) if v_ms > 0.01 else 9999.0,
				"phase_index": float(traci.trafficlight.getPhase(junction_id)),
				"phase_remaining": max(0.0, float(traci.trafficlight.getNextSwitch(junction_id)) - sim_time)
			}
			prob = 0.0
			try:
				prob = float(self.anfis_model.predict_trigger_prob(features)) if self.anfis_model else 0.0
			except Exception:
				prob = 0.0
			# Eşikler: model parametrelerinden
			thr = float(getattr(self.anfis_model, 'params', {}).get('trigger_threshold', 0.5)) if self.anfis_model else 0.5
			near_force = float(getattr(self.anfis_model, 'params', {}).get('near_force_distance_m', 200.0)) if self.anfis_model else 200.0
			# Yakınsa zorla tetikleme eşiği
			if prob <= thr and dist_to_tls <= near_force:
				prob = thr + 1e-3
			return prob > thr
		except Exception:
			return False


