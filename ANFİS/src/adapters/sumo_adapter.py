#!/usr/bin/env python3
"""
SUMO Adapter: TraCI erişimi için sarıcı.
"""

from typing import List, Dict, Tuple, Optional


class SumoAdapter:
	def __init__(self):
		self.connected = False
		self.gui = True

	def connect(self, config_path: str, gui: bool = True) -> bool:
		try:
			import traci
			sumo_bin = "sumo-gui" if gui else "sumo"
			# Otomatik başlamasın: --start vermiyoruz. Delay GUI oynatım gecikmesi içindir.
			cmd = [sumo_bin, "-c", config_path, "--delay", "100"]
			traci.start(cmd)
			self.connected = True
			self.gui = gui
			return True
		except Exception:
			self.connected = False
			return False

	def close(self) -> None:
		try:
			import traci
			if self.connected:
				traci.close()
		finally:
			self.connected = False

	def get_vehicle_ids(self) -> List[str]:
		try:
			import traci
			return list(traci.vehicle.getIDList()) if self.connected else []
		except Exception:
			return []

	def step(self) -> None:
		try:
			import traci
			if self.connected:
				traci.simulationStep()
		except Exception:
			# Bağlantı kapandı veya kullanıcı GUI'yi kapattıysa döngü sonlansın
			self.connected = False

	def get_time(self) -> float:
		try:
			import traci
			return float(traci.simulation.getTime()) if self.connected else 0.0
		except Exception:
			return 0.0

	def get_sim_time(self) -> float:
		"""Simülasyon zamanı (s)."""
		return self.get_time()

	def get_vehicle_edge(self, veh_id: str) -> str:
		try:
			import traci
			return str(traci.vehicle.getRoadID(veh_id))
		except Exception:
			return ""

	def get_vehicle_position(self, veh_id: str):
		try:
			import traci
			return traci.vehicle.getPosition(veh_id)
		except Exception:
			return (0.0, 0.0)

	def get_vehicle_type(self, veh_id: str) -> str:
		try:
			import traci
			return str(traci.vehicle.getTypeID(veh_id))
		except Exception:
			return ""

	def get_traffic_light_ids(self) -> List[str]:
		try:
			import traci
			return list(traci.trafficlight.getIDList()) if self.connected else []
		except Exception:
			return []

	# -------------------- Vehicle helpers --------------------
	def get_vehicle_speed(self, veh_id: str) -> float:
		try:
			import traci
			return float(traci.vehicle.getSpeed(veh_id))
		except Exception:
			return 0.0

	def get_vehicle_angle(self, veh_id: str) -> float:
		"""Returns heading angle in degrees (SUMO convention)."""
		try:
			import traci
			return float(traci.vehicle.getAngle(veh_id))
		except Exception:
			return 0.0

	def get_vehicle_lane_id(self, veh_id: str) -> str:
		try:
			import traci
			return str(traci.vehicle.getLaneID(veh_id))
		except Exception:
			return ""

	def get_vehicle_lane_pos(self, veh_id: str) -> float:
		try:
			import traci
			return float(traci.vehicle.getLanePosition(veh_id))
		except Exception:
			return 0.0

	def get_vehicle_next_tls(self, veh_id: str) -> List[Tuple[str, int, float, str]]:
		"""List of (tlsID, tlsIndex, dist, state) for next controlled TLS along route."""
		try:
			import traci
			return list(traci.vehicle.getNextTLS(veh_id))
		except Exception:
			return []

	# -------------------- Lane helpers --------------------
	def get_lane_vehicle_ids(self, lane_id: str) -> List[str]:
		try:
			import traci
			return list(traci.lane.getLastStepVehicleIDs(lane_id))
		except Exception:
			return []

	def get_lane_halting_number(self, lane_id: str) -> int:
		try:
			import traci
			return int(traci.lane.getLastStepHaltingNumber(lane_id))
		except Exception:
			return 0

	def get_lane_edge_id(self, lane_id: str) -> str:
		try:
			import traci
			return str(traci.lane.getEdgeID(lane_id))
		except Exception:
			return ""

	def get_lane_shape(self, lane_id: str) -> List[Tuple[float, float]]:
		try:
			import traci
			shape = traci.lane.getShape(lane_id)
			return [(float(x), float(y)) for (x, y) in shape]
		except Exception:
			return []

	# -------------------- Junction / TLS helpers --------------------
	def get_junction_position(self, junction_id: str) -> Tuple[float, float]:
		try:
			import traci
			return tuple(traci.junction.getPosition(junction_id))  # type: ignore
		except Exception:
			return (0.0, 0.0)

	def tl_get_state_string(self, tl_id: str) -> str:
		try:
			import traci
			return str(traci.trafficlight.getRedYellowGreenState(tl_id))
		except Exception:
			return ""

	def tl_set_state_string(self, tl_id: str, state: str) -> bool:
		try:
			import traci
			traci.trafficlight.setRedYellowGreenState(tl_id, state)
			return True
		except Exception:
			return False

	def tl_get_num_links(self, tl_id: str) -> int:
		try:
			import traci
			links = traci.trafficlight.getControlledLinks(tl_id)
			return len(links)
		except Exception:
			state = self.tl_get_state_string(tl_id)
			return len(state)

	def tl_get_remaining_phase_time(self, tl_id: str) -> float:
		"""Approx remaining time for current phase (s)."""
		try:
			import traci
			next_switch = float(traci.trafficlight.getNextSwitch(tl_id))
			cur_t = float(traci.simulation.getTime())
			return max(0.0, next_switch - cur_t)
		except Exception:
			return 0.0

	def tl_set_phase_duration(self, tl_id: str, seconds: float) -> bool:
		try:
			import traci
			traci.trafficlight.setPhaseDuration(tl_id, float(seconds))
			return True
		except Exception:
			return False

	def tl_get_program(self, tl_id: str) -> Optional[str]:
		try:
			import traci
			return str(traci.trafficlight.getProgram(tl_id))
		except Exception:
			return None

	def tl_set_program(self, tl_id: str, program_id: str) -> bool:
		try:
			import traci
			traci.trafficlight.setProgram(tl_id, program_id)
			return True
		except Exception:
			return False

	def tl_get_program_states(self, tl_id: str) -> List[str]:
		"""Aktif programın tüm faz durum dizeleri (RYG) listesi."""
		try:
			import traci
			defs = traci.trafficlight.getCompleteRedYellowGreenDefinition(tl_id)
			if not defs:
				state = self.tl_get_state_string(tl_id)
				return [state] if state else []
			logic = defs[0]
			phases = getattr(logic, 'phases', [])
			states = []
			for ph in phases:
				st = getattr(ph, 'state', None)
				if st is not None:
					states.append(str(st))
			return states
		except Exception:
			return []

	def tl_get_phase_index(self, tl_id: str) -> int:
		try:
			import traci
			return int(traci.trafficlight.getPhase(tl_id))
		except Exception:
			return 0

	def tl_get_phase_number(self, tl_id: str) -> int:
		try:
			import traci
			return int(traci.trafficlight.getPhaseNumber(tl_id))
		except Exception:
			# Fallback: infer from state string length (unknown), return small default
			return 1

	def tl_set_phase_index(self, tl_id: str, index: int) -> bool:
		try:
			import traci
			traci.trafficlight.setPhase(tl_id, int(index))
			return True
		except Exception:
			return False

	def tl_get_controlled_links(self, tl_id: str):
		try:
			import traci
			return traci.trafficlight.getControlledLinks(tl_id)
		except Exception:
			return []

	# -------------------- Person / Pedestrian helpers --------------------
	def get_person_ids(self) -> List[str]:
		try:
			import traci
			return list(traci.person.getIDList())
		except Exception:
			return []

	def get_person_position(self, person_id: str) -> Tuple[float, float]:
		try:
			import traci
			pos = traci.person.getPosition(person_id)
			return (float(pos[0]), float(pos[1]))
		except Exception:
			return (0.0, 0.0)

	def count_persons_near(self, x: float, y: float, radius: float = 6.0) -> int:
		try:
			ids = self.get_person_ids()
			if not ids:
				return 0
			import math
			cnt = 0
			for pid in ids:
				px, py = self.get_person_position(pid)
				if math.hypot(px - x, py - y) <= radius:
					cnt += 1
			return cnt
		except Exception:
			return 0

	def get_edge_stats(self) -> Dict[str, Dict[str, float]]:
		"""Kenar bazlı canlı metrikler: araç sayısı, ort. hız, yoğunluk için basit tahmin."""
		stats: Dict[str, Dict[str, float]] = {}
		try:
			if not self.connected:
				return stats
			import traci
			for edge_id in traci.edge.getIDList():
				try:
					veh_n = float(traci.edge.getLastStepVehicleNumber(edge_id))
					mean_v = float(traci.edge.getLastStepMeanSpeed(edge_id))  # m/s
					stats[edge_id] = {"veh": veh_n, "v": mean_v}
				except Exception:
					continue
			return stats
		except Exception:
			return stats

	def get_edges_stats_subset(self, edges: List[str]) -> Dict[str, Dict[str, float]]:
		"""Verilen edge ID listesinin canlı metrikleri (araç sayısı, ort. hız)."""
		stats: Dict[str, Dict[str, float]] = {}
		try:
			if not self.connected:
				return stats
			import traci
			for edge_id in edges:
				try:
					veh_n = float(traci.edge.getLastStepVehicleNumber(edge_id))
					mean_v = float(traci.edge.getLastStepMeanSpeed(edge_id))
					stats[edge_id] = {"veh": veh_n, "v": mean_v}
				except Exception:
					continue
			return stats
		except Exception:
			return stats

	def get_step_length_seconds(self) -> float:
		try:
			import traci
			ms = float(traci.simulation.getDeltaT())  # milliseconds
			return ms / 1000.0
		except Exception:
			return 0.1

	def has_pending(self) -> bool:
		"""Simülasyonda bekleyen araç/durum var mı?"""
		try:
			import traci
			return self.connected and float(traci.simulation.getMinExpectedNumber()) > 0.0
		except Exception:
			return False

	def add_route(self, route_id: str, edges: List[str]) -> bool:
		try:
			import traci
			traci.route.add(route_id, edges)
			return True
		except Exception:
			return False

	def add_vehicle(self, veh_id: str, route_id: str, type_id: str = 'ambulance') -> bool:
		try:
			import traci
			traci.vehicle.add(veh_id, route_id, typeID=type_id)
			return True
		except Exception:
			return False

	def set_route(self, veh_id: str, edges: List[str]) -> bool:
		try:
			import traci
			traci.vehicle.setRoute(veh_id, edges)
			return True
		except Exception:
			return False

	def vehicle_exists(self, veh_id: str) -> bool:
		try:
			return veh_id in self.get_vehicle_ids()
		except Exception:
			return False


