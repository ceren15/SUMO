#!/usr/bin/env python3
"""
Trafik Işığı Denetleyici (iskele)

Amaç: Ambulans yaklaşımı için fazı ayarlamak, minimum/maximum süreleri sağlamak.
İlk sürüm: iskelet metodlar (SUMO entegrasyonu bir sonraki aşamada doldurulacak).
"""

from typing import Optional, Dict, Any, Tuple
import os
import logging

try:
	import xgboost as xgb  # type: ignore
except Exception:
	xgb = None  # type: ignore

logger = logging.getLogger(__name__)


class TrafficLightController:
	def __init__(self, main_junction_id: Optional[str] = None, model_path: Optional[str] = None, action_model_path: Optional[str] = None, extend_model_path: Optional[str] = None):
		self.main_junction_id = main_junction_id
		self.normal_programs: Dict[str, str] = {}
		self.model_path = model_path
		self.model: Optional[Any] = None
		self.action_model: Optional[Any] = None
		self.extend_model: Optional[Any] = None
		self.last_actions: Dict[str, Tuple[float, str]] = {}
		self.last_state_applied: Dict[str, str] = {}
		# Aktif öncelik takibi: tl_id -> {"ambulance_id": str, "state": str}
		self.active_priority: Dict[str, Dict[str, Any]] = {}
		if xgb is not None:
			try:
				# Yeni trigger model yükle
				trigger_model_path = action_model_path or "models/trigger_xgb.json"
				if os.path.exists(trigger_model_path):
					self.action_model = xgb.XGBClassifier()
					self.action_model.load_model(trigger_model_path)
					logger.info(f"Trigger model yüklendi: {trigger_model_path}")
				else:
					logger.warning("Trigger model bulunamadı, fallback sistem kullanılacak")
					self.action_model = None
			except Exception as e:
				logger.warning(f"Model yükleme hatası: {e}")
				self.action_model = None
			# Eski modelleri kaldır
			self.extend_model = None
			self.model = None

	def _list_approach_edges(self, junction_id: str) -> Dict[int, str]:
		"""Kontrol edilen yaklaşım gruplarını indeks→edge_id olarak döndürür."""
		try:
			import traci
			links = traci.trafficlight.getControlledLinks(junction_id)
			edges: Dict[int, str] = {}
			for idx, group in enumerate(links):
				# Aynı idx için birden fazla in_lane olabilir; edge id prefix ortak
				for in_lane, _out_lane, _via in group:
					if "_" in in_lane:
						edge_id = in_lane.split("_")[0]
						edges[idx] = edge_id
						break
			return edges
		except Exception:
			return {}

	def _estimate_eta(self, vehicle_id: str, junction_id: str) -> Tuple[float, float]:
		"""Ambulansın junction'a kalan mesafe ve ETA (s) kaba tahmin."""
		try:
			import traci, math
			# Önce TraCI getNextTLS ile uzaklık (m) bulmayı dene
			try:
				next_tls = traci.vehicle.getNextTLS(vehicle_id)
				for tls_id, _idx, dist, _state in next_tls:
					if str(tls_id) == str(junction_id):
						v = max(1.0, float(traci.vehicle.getSpeed(vehicle_id)))
						return float(dist), float(dist) / v
			except Exception:
				pass
			# Euclidean fallback
			vx, vy = traci.vehicle.getPosition(vehicle_id)
			jx, jy = traci.junction.getPosition(junction_id)
			d = math.hypot(vx - jx, vy - jy)
			v = max(1.0, float(traci.vehicle.getSpeed(vehicle_id)))
			return float(d), float(d) / v
		except Exception:
			return 9999.0, 9999.0

	def _approach_angle_cos(self, vehicle_id: str, junction_id: str) -> float:
		"""Araçtan junction'a yön ile aracın heading'i arasındaki kosinüs benzerliği."""
		try:
			import traci, math
			vx, vy = traci.vehicle.getPosition(vehicle_id)
			jx, jy = traci.junction.getPosition(junction_id)
			dx, dy = (jx - vx), (jy - vy)
			len_v = math.hypot(dx, dy)
			if len_v <= 1e-3:
				return 1.0
			ux, uy = dx / len_v, dy / len_v
			angle_deg = float(traci.vehicle.getAngle(vehicle_id))  # 0=East, CCW
			rad = math.radians(angle_deg)
			vxh, vyh = math.cos(rad), math.sin(rad)
			return float(max(-1.0, min(1.0, ux * vxh + uy * vyh)))
		except Exception:
			return 0.0

	def _extract_features_for_approach(self, junction_id: str, approach_edge_id: str, sim_time: float, vehicle_id: Optional[str]) -> Dict[str, float]:
		"""Genişletilmiş özellik seti (ETA, mesafe, açı dahil)."""
		feats: Dict[str, float] = {}
		try:
			import traci
			phase_index = float(traci.trafficlight.getPhase(junction_id))
			prog_id = str(traci.trafficlight.getProgram(junction_id))
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
			# Faz kalan süresi
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
				"prog_len": float(len(prog_id)),
				"dist_to_junction": float(dist_to_j),
				"eta_to_junction": float(eta_to_j),
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

	def _safe_apply(self, junction_id: str, state_str: str, green_seconds: float) -> bool:
		try:
			import traci
			# Değişim günlüğü: önceki durum ile yeni durum karşılaştır
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
			# Mevcut programa dokunmadan anlık durumu uygula ve faz süresini uzat
			logger.debug(f"[TL] _safe_apply: setting state={state_str} for {green_seconds}s")
			traci.trafficlight.setRedYellowGreenState(junction_id, state_str)
			try:
				traci.trafficlight.setPhaseDuration(junction_id, float(green_seconds))
				logger.debug(f"[TL] _safe_apply: phase duration set to {green_seconds}s")
			except Exception as e:
				logger.debug(f"[TL] _safe_apply: setPhaseDuration failed: {e}")
				# Alternatif: setPhaseDuration yerine setPhase ile deneyelim
				try:
					current_phase = traci.trafficlight.getPhase(junction_id)
					traci.trafficlight.setPhase(junction_id, current_phase)
					logger.debug(f"[TL] _safe_apply: setPhase to {current_phase} as fallback")
				except Exception as e2:
					logger.debug(f"[TL] _safe_apply: setPhase fallback also failed: {e2}")
			# Değişim günlüğü: yeşile dönen ve kırmızıya dönen grupları yaz
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
				# Özet ve detay loglar
				if greens_on or reds_on:
					logger.info(f"[TL] STATE CHANGE tl={junction_id} greens_on={len(greens_on)} reds_on={len(reds_on)} state={state_str}")
					if greens_on:
						logger.info(f"[TL] GREEN_ON tl={junction_id}: " + ", ".join(greens_on))
					if reds_on:
						logger.info(f"[TL] RED_ON tl={junction_id}: " + ", ".join(reds_on))
			except Exception:
				pass
			# Son durumu kaydet
			self.last_state_applied[junction_id] = state_str
			return True
		except Exception as e:
			logger.debug(f"[TL] _safe_apply error: {e}")
			return False

	def set_ambulance_priority(self, traffic_light_id: str, approach_edge_id: str, green_seconds: float = 20.0, ambulance_id: str = None) -> bool:
		"""Ambulansın yönünü yeşil yap, diğer tüm yönleri kırmızı yap."""
		try:
			import traci
			# Mevcut programı sakla
			if traffic_light_id not in self.normal_programs:
				self.normal_programs[traffic_light_id] = str(traci.trafficlight.getProgram(traffic_light_id))
			
			# Ambulansın bulunduğu lane'i al
			ambulance_lane = None
			if ambulance_id:
				try:
					ambulance_lane = traci.vehicle.getLaneID(ambulance_id)
					logger.debug(f"[TL] set_ambulance_priority: ambulance_lane={ambulance_lane}")
				except Exception:
					pass
			
			# Mevcut trafik ışığı durumunu al
			state = list(traci.trafficlight.getRedYellowGreenState(traffic_light_id))
			links = traci.trafficlight.getControlledLinks(traffic_light_id)
			
			# Debug: Tüm controlled links'leri listele
			logger.debug(f"[TL] CONTROLLED LINKS: {len(links)} link var")
			for idx, group in enumerate(links):
				logger.debug(f"[TL] Link {idx}: {len(group)} connection")
				for conn_idx, (in_lane, out_lane, _) in enumerate(group):
					logger.debug(f"[TL]   Conn {conn_idx}: {in_lane} -> {out_lane}")
			
			# Her yön için karar ver: ambulansın yönü yeşil, diğerleri kırmızı
			for idx, group in enumerate(links):
				is_ambulance_direction = False
				for in_lane, out_lane, _ in group:
					# Ambulansın bulunduğu lane'i yeşil yap
					if ambulance_lane and in_lane == ambulance_lane:
						is_ambulance_direction = True
						logger.debug(f"[TL] AMBULANS LANE BULUNDU: {in_lane} == {ambulance_lane}")
						break
					# Approach edge ile eşleşen lane'i yeşil yap
					elif approach_edge_id and in_lane.startswith(approach_edge_id + "_"):
						is_ambulance_direction = True
						logger.debug(f"[TL] APPROACH EDGE BULUNDU: {in_lane} starts with {approach_edge_id}_")
						break
					# Edge ID'si aynı olan lane'leri yeşil yap (lane numarası farklı olabilir)
					elif approach_edge_id and in_lane.split('#')[0] == approach_edge_id.split('#')[0]:
						is_ambulance_direction = True
						logger.debug(f"[TL] EDGE ID EŞLEŞTİ: {in_lane} edge ID matches {approach_edge_id}")
						break
				
				# KARAR: Yeşil mi kırmızı mı?
				state[idx] = 'G' if is_ambulance_direction else 'r'
				logger.debug(f"[TL] Link {idx}: is_ambulance_direction={is_ambulance_direction} -> state[{idx}]={state[idx]}")
			
			# State string'ini oluştur ve uygula
			state_str = ''.join(ch if ch in 'GgYyRr' else 'r' for ch in state)
			
			# Mesafe bilgisini al
			dist_to_tls = float('inf')
			if ambulance_id:
				try:
					next_tls = traci.vehicle.getNextTLS(ambulance_id)
					if next_tls:
						dist_to_tls = float(next_tls[0][2])
				except Exception:
					pass
			
			# Yeşil ve kırmızı sayısını hesapla
			green_count = state_str.count('G') + state_str.count('g')
			red_count = state_str.count('r') + state_str.count('R')
			
			prev_state = self.last_state_applied.get(traffic_light_id)
			log_level = logging.INFO if prev_state != state_str else logging.DEBUG
			logger.log(log_level, f"[TL] TRAFİK IŞIĞI DEĞİŞİMİ: tl={traffic_light_id} mesafe={dist_to_tls:.1f}m | Yeşil: {green_count} yön, Kırmızı: {red_count} yön | State: {state_str}")
			logger.debug(f"[TL] set_ambulance_priority: state={state_str}")
			ok = self._safe_apply(traffic_light_id, state_str, green_seconds)
			if ok:
				# Aktif önceliği kaydet ve bakım döngüsünde yeşili koru
				self.active_priority[traffic_light_id] = {"ambulance_id": ambulance_id, "state": state_str}
			return ok
		except Exception as e:
			logger.debug(f"[TL] set_ambulance_priority error: {e}")
			return False

	def maintain_active_priorities(self, release_distance_m: float = 50.0, keep_green_seconds: float = 1.5) -> None:
		"""Aktif öncelikleri bakımda tut: Ambulans kavşaktan 50m uzaklaşana kadar yeşil kalsın.

		- Her adımda mevcut state'i kısa süreliğine yeniden uygular (fazı canlı tutar)
		- Ambulans kavşak HENÜZ yaklaşımındaysa veya kavşağı yeni geçtiyse (≤ 50m) yeşil korunur
		- Ambulans kavşağı geçtikten SONRA 50m'den fazla uzaklaşınca normal programa döner
		"""
		try:
			import traci, math
			to_restore = []
			for tl_id, info in list(self.active_priority.items()):
				amb_id = str(info.get("ambulance_id") or "")
				state_str = str(info.get("state") or "")
				# Ambulans yoksa geri dön
				try:
					veh_ids = set(traci.vehicle.getIDList())
					if not amb_id or amb_id not in veh_ids:
						to_restore.append(tl_id)
						continue
				except Exception:
					to_restore.append(tl_id)
					continue
				# Mesafeyi ve yaklaşım durumunu ölç
				try:
					vx, vy = traci.vehicle.getPosition(amb_id)
					jx, jy = traci.junction.getPosition(tl_id)
					d = math.hypot(vx - jx, vy - jy)
					# Ambulans halen bu kavşağa doğru mu geliyor?
					is_upcoming = False
					try:
						next_tls = traci.vehicle.getNextTLS(amb_id)
						if next_tls:
							is_upcoming = str(next_tls[0][0]) == str(tl_id)
					except Exception:
						is_upcoming = False
				except Exception:
					# Ölçemezsek emniyetli davran: yeşili sürdürme ve restorasyon listesine ekle
					to_restore.append(tl_id)
					continue
				# Kural:
				# - Eğer ambulans bu kavşağa yaklaşıyorsa (is_upcoming=True) yeşil sürdür
				# - Geçmişse: d <= 50m iken yeşil sürdür, d > 50m olduğunda restore et
				if is_upcoming or (d <= float(release_distance_m)):
					if state_str:
						self._safe_apply(tl_id, state_str, float(keep_green_seconds))
				else:
					to_restore.append(tl_id)
			# Restorasyonları uygula
			for tl_id in to_restore:
				if self.restore(tl_id):
					logger.info(f"[TL] Öncelik sonlandırıldı ve normale döndü: tl={tl_id}")
				self.active_priority.pop(tl_id, None)
		except Exception as e:
			logger.debug(f"[TL] maintain_active_priorities error: {e}")

	def restore(self, junction_id: str) -> bool:
		"""Normal programa dönüş."""
		try:
			import traci
			prog = self.normal_programs.get(junction_id)
			if prog is None:
				return True
			traci.trafficlight.setProgram(junction_id, prog)
			return True
		except Exception:
			return False

	def _extract_features(self, junction_id: str, approach_edge_id: str, sim_time: float) -> Dict[str, float]:
		"""Özellik seti (faz, yaklaşım yükü, hız, halting, faz kalan süre)."""
		feats: Dict[str, float] = {}
		try:
			import traci
			phase_index = float(traci.trafficlight.getPhase(junction_id))
			prog_id = str(traci.trafficlight.getProgram(junction_id))
			veh_total = 0.0
			speed_sum = 0.0
			halt_total = 0.0
			lane_count = 0.0
			links = traci.trafficlight.getControlledLinks(junction_id)
			for _, group in enumerate(links):
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
			# Faz kalan süresi
			try:
				next_sw = float(traci.trafficlight.getNextSwitch(junction_id))
				phase_remaining = max(0.0, next_sw - float(sim_time))
			except Exception:
				phase_remaining = 0.0
			feats = {
				"phase_index": phase_index,
				"veh_approach": veh_total,
				"v_approach": mean_speed,
				"prog_len": float(len(prog_id)),
				"halt_approach": halt_total,
				"phase_remaining": phase_remaining,
			}
		except Exception:
			pass
		return feats

	def _make_approach_green_state(self, junction_id: str, approach_edge_id: str) -> str:
		"""Mevcut state'ten yaklaşım kolunu yeşile çeviren state üret."""
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
		"""XGBoost ile mesafe kontrolü yap - tetiklenmeli mi?"""
		try:
			import traci
			if not ambulance_id:
				logger.debug(f"[TL] No ambulance_id provided")
				return False
			
			# Mesafe ve hız bilgilerini al
			dist_to_tls = float('inf')
			v_ms = 0.0
			queue_len_m = 0.0
			
			try:
				next_tls = traci.vehicle.getNextTLS(ambulance_id)
				if next_tls:
					dist_to_tls = float(next_tls[0][2])
					logger.debug(f"[TL] {ambulance_id} -> next_tls: {next_tls[0][0]}, dist: {dist_to_tls:.1f}m")
			except Exception as e:
				logger.debug(f"[TL] getNextTLS error: {e}")
			
			try:
				v_ms = float(traci.vehicle.getSpeed(ambulance_id))
				logger.debug(f"[TL] {ambulance_id} speed: {v_ms:.1f} m/s")
			except Exception as e:
				logger.debug(f"[TL] getSpeed error: {e}")
			
			# Kuyruk boyu hesapla - ambulansın bulunduğu lane'i de dahil et
			try:
				# Ambulansın bulunduğu lane'i al
				ambulance_lane = None
				try:
					ambulance_lane = traci.vehicle.getLaneID(ambulance_id)
					logger.debug(f"[TL] Ambulance lane: {ambulance_lane}")
				except Exception:
					pass
				
				links_loc = traci.trafficlight.getControlledLinks(junction_id)
				for group in links_loc:
					for in_lane, _out_lane, _via in group:
						# Approach edge ile eşleşen lane'ler
						if approach_edge_id and in_lane.startswith(approach_edge_id + "_"):
							queue_len_m += float(traci.lane.getLastStepVehicleNumber(in_lane)) * 7.5
						# Ambulansın bulunduğu lane (eğer farklıysa)
						elif ambulance_lane and in_lane == ambulance_lane:
							queue_len_m += float(traci.lane.getLastStepVehicleNumber(in_lane)) * 7.5
				logger.debug(f"[TL] Queue length: {queue_len_m:.1f}m (ambulance_lane: {ambulance_lane})")
			except Exception as e:
				logger.debug(f"[TL] Queue calculation error: {e}")
			
			# XGBoost için özellik vektörü oluştur
			features = {
				"dist_to_tls": dist_to_tls,
				"ambulance_speed": v_ms,
				"queue_length": queue_len_m,
				"eta_seconds": (dist_to_tls / max(0.5, v_ms)) if v_ms > 0.01 else 9999.0,
				"phase_index": float(traci.trafficlight.getPhase(junction_id)),
				"phase_remaining": max(0.0, float(traci.trafficlight.getNextSwitch(junction_id)) - sim_time)
			}
			
			logger.debug(f"[TL] Features: {features}")
			
			# XGBoost modeli varsa kullan
			if xgb is not None and self.action_model is not None:
				logger.debug(f"[TL] Using XGBoost model")
				import numpy as np
				# Özellik sırası: dist_to_tls, ambulance_speed, queue_length, eta_seconds, phase_index, phase_remaining
				feature_vector = np.array([[features["dist_to_tls"], features["ambulance_speed"], features["queue_length"], 
											features["eta_seconds"], features["phase_index"], features["phase_remaining"]]])
				# DMatrix yerine direkt numpy array kullan
				prediction = self.action_model.predict(feature_vector)
				
				# Binary classification: 0.5'ten büyükse tetikle
				# XGBoost prediction formatını düzelt
				if hasattr(prediction, 'shape') and len(prediction.shape) > 1:
					trigger_prob = float(prediction[0][0])
				else:
					trigger_prob = float(prediction[0])
				result = trigger_prob > 0.5
				logger.debug(f"[TL] XGBoost prediction: {trigger_prob:.3f}, trigger: {result}")
				
				# Eğer XGBoost tetikleme yapmıyorsa ama ambulans çok yakınsa zorla tetikle
				if not result and dist_to_tls <= 200.0:  # 200m içindeyse zorla tetikle
					logger.debug(f"[TL] XGBoost tetiklemedi ama ambulans yakın ({dist_to_tls:.1f}m), zorla tetikle")
					result = True
				
				return result
			else:
				# Fallback: basit kural tabanlı
				logger.debug(f"[TL] Using fallback rules")
				base_m = 80.0
				k_v = 3.0
				k_q = 0.8
				trigger_distance = max(60.0, min(200.0, base_m + k_v * v_ms + k_q * queue_len_m))
				eta_thresh = 8.0
				result = (dist_to_tls <= trigger_distance) or (features["eta_seconds"] <= eta_thresh)
				logger.debug(f"[TL] Fallback: dist={dist_to_tls:.1f}m <= {trigger_distance:.1f}m OR eta={features['eta_seconds']:.1f}s <= {eta_thresh}s = {result}")
				return result
				
		except Exception as e:
			logger.debug(f"[TL] should_trigger_priority error: {e}")
			return False

	def decide_and_apply(self, junction_id: str, approach_edge_id: str, sim_time: float, min_green: float = 6.0, max_green: float = 20.0, hysteresis_s: float = 4.0) -> str:
		"""XGBoost ile karar ver ve uygula. Dönüş: 'extend' | 'switch' | 'none'"""
		# Histerezis: çok sık karar değiştirmeyi engelle
		last = self.last_actions.get(junction_id)
		if last and (sim_time - last[0]) < hysteresis_s:
			# Yine de eğitim için y_extend=0 loglayalım
			try:
				feats_stub = self._extract_features(junction_id, approach_edge_id, sim_time)
				self._log_training_row(feats_stub, 0.0, junction_id, approach_edge_id, sim_time, action="none")
			except Exception:
				pass
			return "none"
		# Yeni yaklaşım-seçim moduna uygun karar akışı
		cols_basic = ["phase_index", "phase_remaining", "veh_approach", "halt_approach", "v_approach", "prog_len"]
		cols_dir = cols_basic + ["dist_to_junction", "eta_to_junction", "angle_cos"]
		selected_idx = None
		selected_edge = None
		green_seconds = min_green
		try:
			import traci
			veh_ids = traci.vehicle.getIDList()
			emergency_ids = [v for v in veh_ids if any(k in traci.vehicle.getTypeID(v).lower() for k in ("emergency", "ambulance"))]
			primary_veh = emergency_ids[0] if emergency_ids else None
			# Aday yaklaşımlar
			idx_to_edge = self._list_approach_edges(junction_id)
			feat_rows = []
			edges_order = []
			for idx, edge in idx_to_edge.items():
				f = self._extract_features_for_approach(junction_id, edge, sim_time, primary_veh)
				if f:
					feat_rows.append([f.get(c, 0.0) for c in cols_dir])
					edges_order.append((idx, edge, f))
			# Seçim: model varsa her yaklaşım için olasılık tahmin et (binary logistic)
			if xgb is not None and self.action_model is not None and feat_rows:
				import numpy as np
				dm = xgb.DMatrix(np.array(feat_rows, dtype=float), feature_names=cols_dir)
				probs = self.action_model.predict(dm)
				# Çoğu binary modelde shape=(N,1) veya (N,) döner; multi-class ise (N,K)
				if len(getattr(probs, 'shape', [])) == 2 and probs.shape[1] > 1:
					# Multi-class ise her satır için sınıf 1 olasılığını kullan veya ilk sınıfı
					scores = [float(p.max()) for p in probs]
				else:
					scores = [float(p if isinstance(p, (int, float)) else p[0]) for p in probs]
				best_i = int(max(range(len(scores)), key=lambda i: scores[i]))
				selected_idx = edges_order[best_i][0]
				selected_edge = edges_order[best_i][1]
				fsel = edges_order[best_i][2]
				# Süre: regresyon modeli varsa kullan; yoksa ETA/kuyruk tabanlı
				if self.extend_model is not None:
					dm_ex = xgb.DMatrix(np.array([feat_rows[best_i]], dtype=float), feature_names=cols_dir)
					y = float(self.extend_model.predict(dm_ex)[0])
					green_seconds = max(min_green, min(max_green, y))
				else:
					eta = float(fsel.get("eta_to_junction", 6.0))
					vehq = float(fsel.get("veh_approach", 0.0))
					green_seconds = max(min_green, min(max_green, max(6.0, min(eta + 3.0, 12.0)) + min(4.0, vehq)))
			else:
				# Heuristik: yaklaşan kenara en yakın olanı veya en büyük kuyruk
				if edges_order:
					# Öncelik 1: verilen approach_edge_id eşleşmesi
					cand = next(((i, e, f) for (i, e, f) in edges_order if e == approach_edge_id), None)
					if cand is None:
						# Öncelik 2: en yüksek angle_cos ve en düşük ETA
						cand = max(edges_order, key=lambda t: (t[2].get("angle_cos", 0.0), -t[2].get("eta_to_junction", 9999.0)))
					selected_idx, selected_edge, fsel = cand
					eta = float(fsel.get("eta_to_junction", 6.0))
					vehq = float(fsel.get("veh_approach", 0.0))
					green_seconds = max(min_green, min(max_green, max(6.0, min(eta + 3.0, 12.0)) + min(4.0, vehq)))
			# Eğitim logu: oracle etiketi (verilen approach_edge_id) ile tüm yaklaşımları yaz
			for idx, edge, f in edges_order:
				label = 1 if edge == approach_edge_id else 0
				y_ext = green_seconds if (selected_edge == edge) else 0.0
				try:
					self._log_dir_training_row(f, label, y_ext, junction_id, edge, sim_time)
				except Exception:
					pass
		except Exception:
			selected_idx = None
			selected_edge = None
			green_seconds = min_green
		# Emniyet sınırları
		green_seconds = max(min_green, min(max_green, green_seconds))
		# Seçili yaklaşımı uygula: seçilen varsa o grubu yeşil, diğerlerini kırmızı yap
		try:
			import traci
			state = list(traci.trafficlight.getRedYellowGreenState(junction_id))
			links = traci.trafficlight.getControlledLinks(junction_id)
			if selected_idx is not None:
				for idx, group in enumerate(links):
					for _in_lane, _out_lane, _via in group:
						state[idx] = 'G' if idx == selected_idx else 'r'
				state_str = ''.join(ch if ch in 'GgYyRr' else 'r' for ch in state)
				ok = self._safe_apply(junction_id, state_str, green_seconds)
				if ok:
					self.last_actions[junction_id] = (sim_time, "extend")
					self._log_action_console(junction_id, selected_edge or approach_edge_id, sim_time, "extend", green_seconds)
				return "extend"
			# Fallback: eski davranış
			state_str = self._make_approach_green_state(junction_id, approach_edge_id)
			if not state_str:
				state_str = ''.join(ch if ch in 'GgYyRr' else 'r' for ch in state)
			ok = self._safe_apply(junction_id, state_str, green_seconds)
			if ok:
				self.last_actions[junction_id] = (sim_time, "extend")
				self._log_action_console(junction_id, approach_edge_id, sim_time, "extend", green_seconds)
			return "extend"
		except Exception:
			return "none"

	def find_tl_by_approach_edge(self, approach_edge_id: str) -> Optional[str]:
		"""Verilen yaklaşım kenarını kontrol eden trafik ışığını bul."""
		try:
			if not approach_edge_id:
				return None
			import traci
			for tl_id in traci.trafficlight.getIDList():
				try:
					links = traci.trafficlight.getControlledLinks(tl_id)
					for group in links:
						for in_lane, _out_lane, _via in group:
							if in_lane.startswith(approach_edge_id + "_"):
								return tl_id
				except Exception:
					continue
		except Exception:
			return None
		return None

	def _log_training_row(self, feats: Dict[str, float], y_extend: float, junction_id: str, approach_edge_id: str, sim_time: float, action: str = "none") -> None:
		import os, csv
		log_path = os.path.join("data", "signal_training.csv")
		os.makedirs(os.path.dirname(log_path), exist_ok=True)
		row = {**feats, "y_extend": float(y_extend), "action": str(action), "junction_id": junction_id, "approach_edge_id": approach_edge_id, "t": float(sim_time)}
		write_header = not os.path.exists(log_path) or os.path.getsize(log_path) == 0
		with open(log_path, "a", newline="", encoding="utf-8") as f:
			w = csv.DictWriter(f, fieldnames=list(row.keys()))
			if write_header:
				w.writeheader()
			w.writerow(row)

	def _log_action_console(self, junction_id: str, approach_edge_id: str, sim_time: float, action: str, duration: float = 0.0) -> None:
		try:
			import logging
			logging.getLogger("orchestrator").info(f"[TL-Decision] t={sim_time:.1f}s tl={junction_id} edge={approach_edge_id} action={action} dur={duration:.1f}s")
		except Exception:
			pass

