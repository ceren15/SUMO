#!/usr/bin/env python3
"""
Landmark tabanlı Dijkstra ön-hazırlık modülü.

Amaç:
- SUMO network (.net.xml) dosyasından yönlendirme grafiği çıkarmak
- 6-10 adet landmark düğümü seçmek (basit strateji: derece/merkeziyet karması)
- Her landmark için tek-kaynaklı en kısa yol (mesafe/süre) tablolarını üretmek
- A* için admissible alt-sınır: max_i |L_i(goal) - L_i(n)|
"""

import os
import json
import random
import xml.etree.ElementTree as ET
from typing import Dict, List, Tuple


class LandmarkPrecomputer:
	"""Landmark seçimi ve çok-kaynaklı Dijkstra tabloları üretimi"""

	def __init__(self, network_path: str, num_landmarks: int = 8, seed: int = 42):
		self.network_path = network_path
		self.num_landmarks = max(1, num_landmarks)
		random.seed(seed)

		self.nodes: Dict[str, Tuple[float, float]] = {}
		self.out_edges: Dict[str, List[Tuple[str, float]]] = {}

	def _parse_network(self) -> None:
		"""SUMO .net.xml dosyasını okuyup basit yönlü grafiği kurar."""
		root = ET.parse(self.network_path).getroot()
		# Düğümler
		for node in root.findall('.//junction'):
			if node.get('type') == 'internal':
				continue
			jid = node.get('id')
			x = float(node.get('x', '0'))
			y = float(node.get('y', '0'))
			self.nodes[jid] = (x, y)
			self.out_edges.setdefault(jid, [])

		# Kenarlar -> lane hızından süre ağırlığı tahmini
		for edge in root.findall('.//edge'):
			if edge.get('function') in ('internal', 'connector'):
				continue
			from_id = edge.get('from')
			to_id = edge.get('to')
			if from_id not in self.nodes or to_id not in self.nodes:
				continue
			length_sum = 0.0
			speed_sum = 0.0
			lane_count = 0
			for lane in edge.findall('lane'):
				lane_count += 1
				length_sum += float(lane.get('length', '0'))
				speed_sum += float(lane.get('speed', '13.9'))  # ~50km/h default
			if lane_count == 0:
				continue
			avg_len = length_sum / lane_count
			avg_speed = max(0.1, speed_sum / lane_count)
			travel_time = avg_len / avg_speed
			self.out_edges.setdefault(from_id, []).append((to_id, travel_time))

	def _choose_landmarks(self) -> List[str]:
		"""Basit derece merkeziyetine dayalı landmark seçimi."""
		degree = {n: 0 for n in self.nodes.keys()}
		for u, outs in self.out_edges.items():
			degree[u] += len(outs)
			for v, _ in outs:
				degree[v] += 1
		sorted_nodes = sorted(degree.items(), key=lambda kv: kv[1], reverse=True)
		candidates = [n for n, _ in sorted_nodes[: max(self.num_landmarks * 3, self.num_landmarks)]]
		random.shuffle(candidates)
		selected = []
		seen = set()
		for n in candidates:
			if n in seen:
				continue
			selected.append(n)
			seen.add(n)
			if len(selected) >= self.num_landmarks:
				break
		if not selected and self.nodes:
			selected = [next(iter(self.nodes.keys()))]
		return selected

	def _dijkstra(self, source: str) -> Dict[str, float]:
		"""Basit Dijkstra: travel_time ağırlıklarıyla tek-kaynaklı en kısa süre"""
		import heapq
		dist = {n: float('inf') for n in self.nodes.keys()}
		dist[source] = 0.0
		pq = [(0.0, source)]
		while pq:
			du, u = heapq.heappop(pq)
			if du != dist[u]:
				continue
			for v, w in self.out_edges.get(u, []):
				alt = du + w
				if alt < dist[v]:
					dist[v] = alt
					heapq.heappush(pq, (alt, v))
		return dist

	def compute_and_save(self, output_path: str) -> bool:
		"""Landmark tablolarını üretir ve JSON olarak kaydeder."""
		self._parse_network()
		if not self.nodes:
			return False
		landmarks = self._choose_landmarks()
		tables: Dict[str, Dict[str, float]] = {}
		for lm in landmarks:
			tables[lm] = self._dijkstra(lm)
		payload = {
			"meta": {
				"network": os.path.basename(self.network_path),
				"num_nodes": len(self.nodes),
				"num_edges": sum(len(v) for v in self.out_edges.values()),
				"num_landmarks": len(landmarks)
			},
			"landmarks": landmarks,
			"tables": tables
		}
		with open(output_path, 'w', encoding='utf-8') as f:
			json.dump(payload, f)
		return True


