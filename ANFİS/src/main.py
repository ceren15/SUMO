#!/usr/bin/env python3
"""
Akıllı Ambulans Rota & Işık Yönetimi - Orkestratör :Orkestratör: Sistemin beynidir. Ambulans konumunu izler, hangi modüllerin ne zaman çalışacağını belirler.

Komutlar:
  - prep-landmarks: Network'ten landmark tabanlı Dijkstra tablolarını üretir
  - run: (yer tutucu) A* + ANFIS ile çevrimiçi simülasyonu çalıştırır
"""

import os
import sys
import argparse
import logging

# Yerel modüller (paket-içi)
from src.offline.landmarks import LandmarkPrecomputer


def setup_logging() -> logging.Logger:
	"""Basit log yapılandırması"""
	log_dir = "logs"
	os.makedirs(log_dir, exist_ok=True)
	logging.basicConfig(
		level=logging.DEBUG,  # DEBUG seviyesine çıkar
		format='%(asctime)s [%(levelname)s] %(name)s: %(message)s',
		handlers=[logging.StreamHandler()]
	)
	return logging.getLogger("orchestrator")


def _compute_replan_in_process(net_path: str, landmark_path: str, start_node: str, goal_node: str, edge_stats_snapshot: dict):
	"""A* yeniden planlama (ayrı süreçte, GIL bloklamasız)."""
	from src.online.router import OnlineRouter
	# ANFIS kancaları süreç içinde no-op kalsın
	router = OnlineRouter(
		network_path=net_path,
		landmark_json_path=landmark_path,
		get_live_edge_factor=lambda e: 1.0,
		get_signal_delay=lambda n: 0.0,
		anfis_adjust_heuristic=lambda h, ctx: h,
	)
	def live_factor(edge_id: str) -> float:
		base_t = getattr(router, 'edge_base_time', {}).get(edge_id, 0.0)
		if base_t <= 0:
			return 1.0
		st = edge_stats_snapshot.get(edge_id)
		if not st:
			return 1.0
		veh = st.get("veh", 0.0)
		v = st.get("v", getattr(router, 'edge_free_speed', {}).get(edge_id, 10.0))
		v_ref = max(1.0, getattr(router, 'edge_free_speed', {}).get(edge_id, 10.0))
		cong = max(0.0, min(3.0, (v_ref / max(1.0, v))))
		load = 1.0 + min(2.0, veh / 20.0)
		return max(1.0, min(5.0, 0.5 * cong + 0.5 * load))
	orig_live = router.get_live_edge_factor
	router.get_live_edge_factor = live_factor
	best_time, best_path = router.astar(start_node, goal_node)
	router.get_live_edge_factor = orig_live
	return best_time, best_path, edge_stats_snapshot


class IncrementalAStar:
	"""A*'ı adım adım çalıştırmak için artımlı arama (ana döngüyü bloklamaz)."""
	def __init__(self, router, start_node: str, goal_node: str, edge_stats_snapshot: dict):
		import heapq
		self.router = router
		self.start = start_node
		self.goal = goal_node
		self.edge_stats = edge_stats_snapshot
		self.heapq = heapq
		self.open_pq = []
		self.heapq.heappush(self.open_pq, (0.0, start_node))
		self.g_score = {start_node: 0.0}
		self.parent = {start_node: None}
		self.done = False
		self.result = (float('inf'), [])
	def _live_factor(self, edge_id: str) -> float:
		base_t = getattr(self.router, 'edge_base_time', {}).get(edge_id, 0.0)
		if base_t <= 0:
			return 1.0
		st = self.edge_stats.get(edge_id)
		if not st:
			return 1.0
		veh = st.get("veh", 0.0)
		v = st.get("v", getattr(self.router, 'edge_free_speed', {}).get(edge_id, 10.0))
		v_ref = max(1.0, getattr(self.router, 'edge_free_speed', {}).get(edge_id, 10.0))
		cong = max(0.0, min(3.0, (v_ref / max(1.0, v))))
		load = 1.0 + min(2.0, veh / 20.0)
		return max(1.0, min(5.0, 0.5 * cong + 0.5 * load))
	def step(self, max_expansions: int = 500) -> None:
		if self.done:
			return
		expanded = 0
		while self.open_pq and expanded < max_expansions:
			_, u = self.heapq.heappop(self.open_pq)
			if u == self.goal:
				path = []
				cur = self.goal
				while cur is not None:
					path.append(cur)
					cur = self.parent.get(cur)
				path.reverse()
				self.result = (self.g_score[self.goal], path)
				self.done = True
				return
			for v, base_time, edge_id in self.router.out_edges.get(u, []):
				live = self._live_factor(edge_id)
				cand_g = self.g_score[u] + base_time * max(0.1, float(live))
				cand_g += max(0.0, float(self.router.get_signal_delay(v)))
				if cand_g < self.g_score.get(v, float('inf')):
					self.g_score[v] = cand_g
					self.parent[v] = u
					h = self.router.heuristic(v, self.goal, context={"g": cand_g})
					self.heapq.heappush(self.open_pq, (cand_g + h, v))
			expanded += 1
		if not self.open_pq:
			self.done = True
			self.result = (float('inf'), [])
	def finished(self) -> bool:
		return self.done
	def get_result(self):
		return self.result

def cmd_prep_landmarks(args) -> int:
	"""Offline Dijkstra (landmark) ön-hazırlığı çalıştır"""
	logger = setup_logging()
	logger.info("Offline landmark ön-hazırlık başlıyor…")

	net_path = args.net
	if not os.path.exists(net_path):
		logger.error(f"Network dosyası bulunamadı: {net_path}")
		return 1

	output_dir = os.path.dirname(args.output)
	if output_dir:
		os.makedirs(output_dir, exist_ok=True)

	pre = LandmarkPrecomputer(
		network_path=net_path,
		num_landmarks=args.num_landmarks,
		seed=args.seed
	)
	result = pre.compute_and_save(args.output)
	if result:
		logger.info(f"Landmark tabloları oluşturuldu: {args.output}")
		return 0
	else:
		logger.error("Landmark hesaplama başarısız")
		return 1


def cmd_run(args) -> int:
	"""Online A* + ANFIS akışını başlatır (ilk sürüm: rota hesapla ve logla)."""
	logger = setup_logging()

	# Bileşenler
	from src.online.router import OnlineRouter
	from src.controllers import TrafficLightController

	net_path = "config/network_with_tl.net.xml"
	landmark_path = "data/landmarks.json"
	if not os.path.exists(net_path):
		logger.error(f"Network dosyası yok: {net_path}. Önce dönüştürme scriptini çalıştırın.")
		return 1
	if not os.path.exists(landmark_path):
		logger.error(f"Landmark dosyası yok: {landmark_path}. 'prep-landmarks' komutunu çalıştırın.")
		return 1

	router = OnlineRouter(
		network_path=net_path,
		landmark_json_path=landmark_path,
		get_live_edge_factor=lambda edge_id: 1.0,
		get_signal_delay=lambda node_id: 0.0,
		anfis_adjust_heuristic=lambda base_h, ctx: base_h,
	)

	# Başlangıç/hedef düğümleri belirle
	start = args.start_node
	goal = args.goal_node
	if not start or not goal:
		# deterministik iki düğüm seçimi (örnek)
		nodes = list(router.nodes.keys())
		if len(nodes) < 2:
			logger.error("Network çok küçük veya düğüm bulunamadı.")
			return 1
		start = start or nodes[0]
		goal = goal or nodes[-1]

	logger.info(f"A* rota hesaplanıyor: start={start} → goal={goal}")
	total_time, path_nodes = router.astar(start, goal)
	if total_time == float('inf') or not path_nodes:
		logger.error("Rota bulunamadı.")
		return 1
	logger.info(f"Rota bulundu. Süre (tahmini): {total_time:.1f} s, Düğümler: {len(path_nodes)}")
	logger.info("Örnek rota kesiti: " + " → ".join(path_nodes[:10]) + (" → …" if len(path_nodes) > 10 else ""))

	# SUMO entegrasyonu: bağlan, ambulans için otomatik rota oluştur ve replan yap
	if not args.dry_run:
		try:
			from src.adapters import SumoAdapter
			adapter = SumoAdapter()
			if not adapter.connect(args.config, gui=args.gui):
				logger.warning("SUMO bağlantısı başarısız; sadece rota hesaplandı.")
				return 0
			replan_interval = float(getattr(args, 'replan_interval', 10.0))
			logger.info(f"SUMO bağlantısı kuruldu. {replan_interval:.0f} saniyede bir yeniden planlama çalışacak.")
			# ANFIS tabanlı trafik ışığı kontrolcüsü
			tl_ids = adapter.get_traffic_light_ids()
			main_tl = tl_ids[0] if tl_ids else None
			tlc = TrafficLightController(main_tl, anfis_model_path=getattr(args, 'anfis_model', None))
			import time
			from threading import Thread, Lock
			from concurrent.futures import ProcessPoolExecutor
			router_lock = Lock()
			acc = 0.0
			loops = 0
			# Hastane hedefi: CLI > sabit ID > fallback
			DEFAULT_HOSPITAL = "cluster_6762197026_6762197027_6762197028_6762197029"
			goal_node = goal or DEFAULT_HOSPITAL
			if goal_node not in router.nodes:
				# fallback: önceki kestirim
				cands = [nid for nid in router.nodes.keys() if nid.startswith('cluster_9855125')]
				if cands:
					goal_node = cands[0]
				else:
					goal_node = list(router.nodes.keys())[-1]

			spawn_period = max(5.0, float(args.spawn_period))
			spawn_acc = 0.0
			last_spawn_sim_t = 0.0
			spawn_seq = 0
			# İlk ambulansı hemen oluştur (kullanıcı beklemeden görsün)
			import random
			nodes_list_boot = router.nodes_reaching(goal_node) or list(router.nodes.keys())
			if nodes_list_boot:
				start_node_boot = random.choice(nodes_list_boot)
				_, boot_path = router.astar(start_node_boot, goal_node)
				edges_boot = []
				for i in range(len(boot_path)-1):
					u2, v2 = boot_path[i], boot_path[i+1]
					ed = router.endpoints_to_edge.get((u2, v2))
					if ed:
						edges_boot.append(ed)
				if edges_boot:
					rid0 = f"amb_route_{spawn_seq}"
					vid0 = f"ambulance_{spawn_seq}"
					spawn_seq += 1
					adapter.add_route(rid0, edges_boot)
					adapter.add_vehicle(vid0, rid0, type_id='ambulance')
					logger.info(f"İlk ambulans: {vid0}, from={start_node_boot} → {goal_node}, edges={len(edges_boot)}")
			# SUMO bekleyen olduğu sürece çalış; ayrıca güvenlik için üst sınır
			max_loops = 1000000
			last_replan_sim_t = -1.0
			# Asenkron replan durumu
			replan_thread = None
			replan_in_flight = False
			replan_result = None  # tuple(best_time, best_path, cur_t)
			replan_future = None
			executor = None  # Process pool kaldırıldı
			incr_search = None  # IncrementalAStar durumu
			max_sim_time = getattr(args, 'max_sim_time', None)
			# Eski kontrolcü kaldırıldı; doğrudan TL kontrolcüsü kullanılacak

			while adapter.connected and loops < max_loops:
				adapter.step()
				loops += 1
				acc += adapter.get_step_length_seconds()
				spawn_acc += adapter.get_step_length_seconds()
				# Yeşil öncelik bakımını her adımda çalıştır
				try:
					tlc.maintain_active_priorities(release_distance_m=50.0, keep_green_seconds=1.5)
				except Exception:
					pass
				# Periyodik ambulans spawn (simülasyon zamanına göre)
				cur_t = adapter.get_sim_time()
				if max_sim_time is not None and cur_t >= float(max_sim_time):
					break
				if (cur_t - last_spawn_sim_t) >= spawn_period and cur_t > 0:
					spawn_acc = 0.0
					last_spawn_sim_t = cur_t
					# rastgele bir başlangıç düğümü seç
					import random
					nodes_list = router.nodes_reaching(goal_node) or list(router.nodes.keys())
					start_node_spawn = random.choice(nodes_list)
					# spawn rotasını her zaman hastaneye (goal_node) yap
					_, spawn_path = router.astar(start_node_spawn, goal_node)
					edges_spawn = []
					for i in range(len(spawn_path)-1):
						u2, v2 = spawn_path[i], spawn_path[i+1]
						ed = router.endpoints_to_edge.get((u2, v2))
						if ed:
							edges_spawn.append(ed)
					if edges_spawn:
						rid = f"amb_route_{spawn_seq}"
						vid = f"ambulance_{spawn_seq}"
						spawn_seq += 1
						if adapter.connected:
							adapter.add_route(rid, edges_spawn)
							adapter.add_vehicle(vid, rid, type_id='ambulance')
						logger.info(f"Yeni ambulans: {vid}, from={start_node_spawn} → {goal_node}, edges={len(edges_spawn)}")
				# Replan (sabit aralık: args.replan_interval) — asenkron hesaplama
				if last_replan_sim_t < 0:
					last_replan_sim_t = cur_t
				# Ambulans seçimi ve snap-to-node
				veh_ids = adapter.get_vehicle_ids()
				ambulance_id = next((v for v in veh_ids if any(k in adapter.get_vehicle_type(v).lower() for k in ("emergency", "ambulance"))), None)
				start_node = start
				if ambulance_id:
					x, y = adapter.get_vehicle_position(ambulance_id)
					snapped = router.nearest_node(x, y)
					if snapped:
						start_node = snapped
					# ANFIS ile tetikleme ve öncelik uygula
					cand_tl_id = None
					approach_edge = adapter.get_vehicle_edge(ambulance_id)
					dist_to_tls = float('inf')
					try:
						import traci
					except Exception:
						traci = None  # type: ignore
					if traci is not None:
						try:
							next_tls = traci.vehicle.getNextTLS(ambulance_id)
							if next_tls:
								cand_tl_id = str(next_tls[0][0])
								dist_to_tls = float(next_tls[0][2])
						except Exception:
							cand_tl_id = None
					if (cand_tl_id is None) and approach_edge:
						tl_list = []
						if traci is not None:
							try:
								tl_list = adapter.get_traffic_light_ids()
							except Exception:
								tl_list = []
						for tl_id in tl_list:
							try:
								links = traci.trafficlight.getControlledLinks(tl_id) if traci is not None else []
								for group in links:
									for in_lane, _out_lane, _via in group:
										if in_lane.startswith(approach_edge + "_"):
											cand_tl_id = tl_id
											dist_to_tls = 150.0
											break
									if cand_tl_id:
										break
							except Exception:
								continue
					should_trigger = False
					if cand_tl_id and approach_edge:
						should_trigger = tlc.should_trigger_priority(cand_tl_id, approach_edge, cur_t, ambulance_id)
					if cand_tl_id and approach_edge and should_trigger:
						logger.info(f"[TL] (ANFIS) approach={approach_edge} -> tl={cand_tl_id} karar uygulanıyor (veh={ambulance_id})")
						try:
							ok = tlc.set_ambulance_priority(cand_tl_id, approach_edge, green_seconds=12.0, ambulance_id=ambulance_id)
							if ok:
								logger.info(f"[Priority] t={cur_t:.1f}s veh={ambulance_id} tl={cand_tl_id} edge={approach_edge} action=green_priority")
						except Exception as e:
							logger.debug(f"[Priority] set_ambulance_priority error: {e}")
							pass
				# Zaman temelli tetikleme: asenkron replan başlat
				if (cur_t - last_replan_sim_t) >= replan_interval and cur_t > 0 and not replan_in_flight:
					last_replan_sim_t = cur_t
					# Yakın çevredeki kenarlar için sınırlı canlı metrik al (tam ağ yerine)
					def collect_local_edges(seed_node: str, max_depth: int = 2, max_edges: int = 200):
						from collections import deque
						seen = set([seed_node])
						q = deque([(seed_node, 0)])
						edges = []
						while q and len(edges) < max_edges:
							n, d = q.popleft()
							for v, base_time, eid in router.out_edges.get(n, []):
								if eid:
									edges.append(eid)
								if d < max_depth and v not in seen:
									seen.add(v)
									q.append((v, d+1))
						return edges[:max_edges]
					edges_subset = collect_local_edges(start_node, max_depth=2, max_edges=200)
					edge_stats_snapshot = adapter.get_edges_stats_subset(edges_subset) if edges_subset else {}
					# Artımlı A* başlat (bloklamadan, her adımda sınırlı genişleme)
					from src.online.router import OnlineRouter
					incr_search = IncrementalAStar(router, start_node, goal_node, edge_stats_snapshot)
					replan_in_flight = True
				# Replan sonucu hazırsa işle ve logla (bloklamadan)
				if incr_search is not None and replan_in_flight:
					# Her döngüde sınırlı sayıda düğüm genişlet; simülasyon akışı durmaz
					incr_search.step(max_expansions=50)
					if incr_search.finished():
						res_time, res_path = incr_search.get_result()
						if res_time == float('inf') or not res_path:
							# başarısız arama, sonucu atla
							incr_search = None
							replan_in_flight = False
							continue
						best_time, best_path = res_time, res_path
						edge_stats_used = incr_search.edge_stats
						t_mark = cur_t
						incr_search = None
						replan_in_flight = False
						# Logla
						def lf_used(edge_id: str) -> float:
							if not edge_id:
								return 1.0
							base_t = getattr(router, 'edge_base_time', {}).get(edge_id, 0.0)
							if base_t <= 0:
								return 1.0
							st = edge_stats_used.get(edge_id)
							if not st:
								return 1.0
							veh = st.get("veh", 0.0)
							v = st.get("v", getattr(router, 'edge_free_speed', {}).get(edge_id, 10.0))
							v_ref = max(1.0, getattr(router, 'edge_free_speed', {}).get(edge_id, 10.0))
							cong = max(0.0, min(3.0, (v_ref / max(1.0, v))))
							load = 1.0 + min(2.0, veh / 20.0)
							return max(1.0, min(5.0, 0.5 * cong + 0.5 * load))
						# ALT-KIYAS
						if len(best_path) >= 3:
							uA, wA = best_path[0], best_path[1]
							uB, wB = best_path[0], best_path[2]
							edgeA = router.endpoints_to_edge.get((uA, wA))
							edgeB = router.endpoints_to_edge.get((uB, wB))
							def est(edge_id: str) -> float:
								base = getattr(router, 'edge_base_time', {}).get(edge_id, 0.0)
								live = lf_used(edge_id)
								return base * live
							if edgeA and edgeB:
								logger.info(
									f"[ALT-KIYAS] edgeA={edgeA} t~{est(edgeA):.2f}s vs edgeB={edgeB} t~{est(edgeB):.2f}s → seçilen={edgeA}"
								)
						# İlk 3 kenarın kalemleri
						if len(best_path) >= 2:
							items = []
							for i in range(min(3, len(best_path)-1)):
								u2, v2 = best_path[i], best_path[i+1]
								eid = router.endpoints_to_edge.get((u2, v2))
								if not eid:
									continue
								bt = getattr(router, 'edge_base_time', {}).get(eid, 0.0)
								lfv = lf_used(eid)
								items.append(f"{eid}: base={bt:.2f}s live={lfv:.2f} adj={bt*lfv:.2f}s")
							if items:
								logger.info("[Edges] " + " | ".join(items))
						logger.info(f"[Replan] t={t_mark:.1f}s ETA~{best_time:.1f}s, düğüm: {len(best_path)}")
						# (Öncelik uygulaması yukarıya taşındı; replan sonucuna bağlı olmadan her döngüde çalışır)
			adapter.close()
		except Exception as e:
			logger.warning(f"SUMO entegrasyonu sırasında hata: {e}")

	return 0


def build_arg_parser() -> argparse.ArgumentParser:
	parser = argparse.ArgumentParser(description="Akıllı Ambulans - Orkestratör")
	sub = parser.add_subparsers(dest="command", required=True)

	# prep-landmarks
	prep = sub.add_parser("prep-landmarks", help="ALT için landmark tablolarını üret")
	prep.add_argument("--net", default="config/network_with_tl.net.xml", help="SUMO network .net.xml yolu")
	prep.add_argument("--output", default="data/landmarks.json", help="Çıktı dosyası")
	prep.add_argument("--num-landmarks", type=int, default=8, help="Landmark sayısı (6-10 arası önerilir)")
	prep.add_argument("--seed", type=int, default=42, help="Rastgelelik tekrarlanabilirliği için tohum")
	prep.set_defaults(func=cmd_prep_landmarks)

	# run
	run = sub.add_parser("run", help="Simülasyonu çalıştır (A* + ANFIS)")
	run.add_argument("--config", default="config/simulation.sumocfg", help="SUMO .sumocfg")
	run.add_argument("--gui", action="store_true", help="GUI modunda çalıştır")
	run.add_argument("--dry-run", action="store_true", help="SUMO'ya bağlanmadan sadece rota hesapla")
	run.add_argument("--start-node", default=None, help="Başlangıç junction ID")
	run.add_argument("--goal-node", default="cluster_6762197026_6762197027_6762197028_6762197029", help="Hedef (hastane) junction ID")
	run.add_argument("--spawn-period", type=float, default=60.0, help="Ambulans spawn periyodu (s)")
	run.add_argument("--replan-interval", type=float, default=10.0, help="Yeniden planlama periyodu (s)")
	run.add_argument("--max-sim-time", type=float, default=None, help="Maksimum simülasyon süresi (s) – aşılınca çıkılır")
	run.add_argument("--anfis-model", default="models/anfis.json", help="ANFIS model dosyası (.json)")
	run.set_defaults(func=cmd_run)

	return parser


def main():
	parser = build_arg_parser()
	args = parser.parse_args()
	return args.func(args)


if __name__ == "__main__":
	sys.exit(main())


