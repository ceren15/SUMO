#!/usr/bin/env python3
"""
Online A* Rotalayıcı (landmark tabanlı alt-sınır + ANFIS düzeltme için kancalar)
"""

import json
import xml.etree.ElementTree as ET
from typing import Dict, List, Tuple, Callable, Optional
import math


class OnlineRouter:
	"""A* yönlendirme motoru (gerçek zamanlı)

	- g(n): temel süre (mesafe/hız) + canlı trafik katsayısı + ışık gecikmesi (tahmin)
	- h(n): landmark alt-sınırı (admissible) + ANFIS düzeltme (konservatif kanca)
	"""

	def __init__(
		self,
		network_path: str,
		landmark_json_path: str,
		get_live_edge_factor: Optional[Callable[[str], float]] = None,
		get_signal_delay: Optional[Callable[[str], float]] = None,
		anfis_adjust_heuristic: Optional[Callable[[float, Dict], float]] = None,
	):
		self.network_path = network_path
		self.landmark_json_path = landmark_json_path
		self.get_live_edge_factor = get_live_edge_factor or (lambda edge_id: 1.0)
		self.get_signal_delay = get_signal_delay or (lambda node_id: 0.0)
		self.anfis_adjust_heuristic = anfis_adjust_heuristic or (lambda base_h, ctx: base_h)

		self.nodes: Dict[str, Tuple[float, float]] = {}
		self.out_edges: Dict[str, List[Tuple[str, float, str]]] = {}  # u -> [(v, base_time, edge_id)]
		self.in_neighbors: Dict[str, List[str]] = {}                  # v -> [u]
		self.edge_length: Dict[str, float] = {}
		self.edge_free_speed: Dict[str, float] = {}
		self.edge_base_time: Dict[str, float] = {}
		self.edge_to_endpoints: Dict[str, Tuple[str, str]] = {}       # edge_id -> (u, v)
		self.endpoints_to_edge: Dict[Tuple[str, str], str] = {}       # (u, v) -> edge_id
		self.landmarks: List[str] = []
		self.tables: Dict[str, Dict[str, float]] = {}

		self._parse_network()
		self._load_landmarks()

	def nearest_node(self, x: float, y: float) -> Optional[str]:
		"""Verilen SUMO düzlemi (x,y) için en yakın düğüm ID'si."""
		best_id: Optional[str] = None
		best_d = float('inf')
		for nid, (nx, ny) in self.nodes.items():
			d = (nx - x) * (nx - x) + (ny - y) * (ny - y)
			if d < best_d:
				best_d = d
				best_id = nid
		return best_id

	def nodes_reaching(self, goal: str) -> List[str]:
		"""Hedefe ulaşabilen düğümler (geri komşuluk ile)."""
		if goal not in self.nodes:
			return []
		seen: Dict[str, bool] = {goal: True}
		stack: List[str] = [goal]
		while stack:
			v = stack.pop()
			for u in self.in_neighbors.get(v, []):
				if u not in seen:
					seen[u] = True
					stack.append(u)
		return list(seen.keys())

	def _parse_network(self) -> None:
		root = ET.parse(self.network_path).getroot()
		for node in root.findall('.//junction'):
			if node.get('type') == 'internal':
				continue
			jid = node.get('id')
			x = float(node.get('x', '0'))
			y = float(node.get('y', '0'))
			self.nodes[jid] = (x, y)
			self.out_edges.setdefault(jid, [])
		for edge in root.findall('.//edge'):
			if edge.get('function') in ('internal', 'connector'):
				continue
			u = edge.get('from')
			v = edge.get('to')
			if u not in self.nodes or v not in self.nodes:
				continue
			length_sum = 0.0
			speed_sum = 0.0
			lane_count = 0
			for lane in edge.findall('lane'):
				lane_count += 1
				length_sum += float(lane.get('length', '0'))
				speed_sum += float(lane.get('speed', '13.9'))
			if lane_count == 0:
				continue
			avg_len = length_sum / lane_count
			avg_speed = max(0.1, speed_sum / lane_count)
			base_time = avg_len / avg_speed
			edge_id = edge.get('id', f"{u}>{v}")
			self.out_edges.setdefault(u, []).append((v, base_time, edge_id))
			self.in_neighbors.setdefault(v, []).append(u)
			self.edge_length[edge_id] = avg_len
			self.edge_free_speed[edge_id] = avg_speed
			self.edge_base_time[edge_id] = base_time
			self.edge_to_endpoints[edge_id] = (u, v)
			self.endpoints_to_edge[(u, v)] = edge_id

	def _load_landmarks(self) -> None:
		with open(self.landmark_json_path, 'r', encoding='utf-8') as f:
			data = json.load(f)
		self.landmarks = data.get('landmarks', [])
		self.tables = data.get('tables', {})

	def heuristic(self, node: str, goal: str, context: Optional[Dict] = None) -> float:
		"""ALT: max_i |L_i(goal) - L_i(node)|; ardından ANFIS kancası ile konservatif ayar.

		Not: Varsayılan ANFIS kancası base_h'i değiştirmez. Admissible kalmak için
		kullanıcı kancası 'asla düşürmeyecek' ve teorik alt-sınırı aşmayacak şekilde tasarlanmalıdır.
		"""
		context = context or {}
		base = 0.0
		for lm in self.landmarks:
			lm_table = self.tables.get(lm, {})
			g_goal = lm_table.get(goal, float('inf'))
			g_node = lm_table.get(node, float('inf'))
			if g_goal == float('inf') or g_node == float('inf'):
				continue
			base = max(base, abs(g_goal - g_node))
		return float(self.anfis_adjust_heuristic(base, {**context, "node": node, "goal": goal}))

	def astar(self, start: str, goal: str) -> Tuple[float, List[str]]:
		"""A* ile start→goal rota üretir; (toplam_süre, düğüm_listesi) döner."""
		import heapq
		open_pq: List[Tuple[float, str]] = []
		heapq.heappush(open_pq, (0.0, start))
		g_score: Dict[str, float] = {start: 0.0}
		parent: Dict[str, Optional[str]] = {start: None}

		while open_pq:
			_, u = heapq.heappop(open_pq)
			if u == goal:
				# yol oluştur
				path = []
				cur = goal
				while cur is not None:
					path.append(cur)
					cur = parent.get(cur)
				path.reverse()
				return g_score[goal], path
			for v, base_time, edge_id in self.out_edges.get(u, []):
				live = self.get_live_edge_factor(edge_id)
				cand_g = g_score[u] + base_time * max(0.1, float(live))
				# sinyal gecikmesini düğümde uygula (hedef düğüme girişte)
				cand_g += max(0.0, float(self.get_signal_delay(v)))
				if cand_g < g_score.get(v, float('inf')):
					g_score[v] = cand_g
					parent[v] = u
					h = self.heuristic(v, goal, context={"g": cand_g})
					heapq.heappush(open_pq, (cand_g + h, v))
		return float('inf'), []


