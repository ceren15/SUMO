#!/usr/bin/env python3
"""
SUMO Adapter: TraCI erişimi için sarıcı.
"""

from typing import List, Dict


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


