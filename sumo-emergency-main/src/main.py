#!/usr/bin/env python3
"""
Akıllı Ambulans Rota & Işık Yönetimi - Orkestratör

Komutlar:
  - prep-landmarks: Network'ten landmark tabanlı Dijkstra tablolarını üretir
  - run: A* + (kural tabanlı / opsiyonel ANFIS) ile çevrimiçi simülasyonu çalıştırır
"""

import os
import sys
import argparse
import logging
import subprocess

# Paket içi modüller
from src.offline.landmarks import LandmarkPrecomputer
from src.controllers.traffic_light import TrafficLightController, TLSMap
from src.adapters.sumo_adapter import SumoAdapter
from src.ai.anfis import ANFIS


# ------------------------ TLS Map Otomatik Çıkarıcı ------------------------ #
def _build_tls_map_auto(tl_id: str | None) -> TLSMap | None:
    if not tl_id:
        return None
    import traci
    defs = traci.trafficlight.getCompleteRedYellowGreenDefinition(tl_id)
    phases = defs[0].phases if defs else []
    if not phases:
        return None

    # En çok 'G' içeren 2 fazı ana/yan kabul et
    g_counts = sorted(
        [(i, p.state.count('G') + p.state.count('g')) for i, p in enumerate(phases)],
        key=lambda t: t[1], reverse=True
    )
    main_green = [g_counts[0][0]]
    side_green = [g_counts[1][0]] if len(g_counts) > 1 else main_green

    links = traci.trafficlight.getControlledLinks(tl_id)

    def lanes_of_phase(pi: int):
        lanes = set()
        st = phases[pi].state
        for si, ch in enumerate(st):
            if ch in "Gg" and si < len(links):
                for in_lane, _, _ in links[si]:
                    lanes.add(in_lane)
        return list(lanes)

    lanes_main = lanes_of_phase(main_green[0])[:10]
    lanes_side = [l for l in lanes_of_phase(side_green[0]) if l not in lanes_main][:10]

    return TLSMap(
        tls_id=tl_id,
        main_green_phases=main_green,
        side_green_phases=side_green,
        lanes_main=lanes_main,
        lanes_side=lanes_side,
        min_green=5, max_green=40, extend_clip=(0.0, 10.0),
    )


# ------------------------------- Logging ----------------------------------- #
def setup_logging() -> logging.Logger:
    os.makedirs("logs", exist_ok=True)
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s [%(levelname)s] %(name)s: %(message)s',
        handlers=[logging.StreamHandler()]
    )
    return logging.getLogger("orchestrator")


# ----------------------------- Artımlı A* ---------------------------------- #
class IncrementalAStar:
    """A*'ı adım adım çalıştırmak için artımlı arama."""
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


# --------------------------------- Komutlar -------------------------------- #
def cmd_prep_landmarks(args) -> int:
    logger = setup_logging()
    logger.info("Offline landmark ön-hazırlık başlıyor…")
    net_path = args.net
    if not os.path.exists(net_path):
        logger.error(f"Network dosyası bulunamadı: {net_path}")
        return 1
    out_dir = os.path.dirname(args.output)
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)
    pre = LandmarkPrecomputer(network_path=net_path, num_landmarks=args.num_landmarks, seed=args.seed)
    result = pre.compute_and_save(args.output)
    if result:
        logger.info(f"Landmark tabloları oluşturuldu: {args.output}")
        return 0
    else:
        logger.error("Landmark hesaplama başarısız")
        return 1


def cmd_run(args) -> int:
    logger = setup_logging()

    from src.online.router import OnlineRouter

    net_path = "config/network_with_tl.net.xml"
    landmark_path = "data/landmarks.json"
    if not os.path.exists(net_path):
        logger.error(f"Network dosyası yok: {net_path}.")
        return 1
    if not os.path.exists(landmark_path):
        logger.error(f"Landmark dosyası yok: {landmark_path}. 'prep-landmarks' komutunu çalıştırın.")
        return 1

    anfis = ANFIS()
    router = OnlineRouter(
        network_path=net_path,
        landmark_json_path=landmark_path,
        get_live_edge_factor=lambda edge_id: 1.0,
        get_signal_delay=lambda node_id: anfis.signal_delay(node_id, {}),
        anfis_adjust_heuristic=lambda base_h, ctx: anfis.adjust_heuristic(base_h, ctx),
    )

    # Başlangıç/hedef
    start = args.start_node
    goal = args.goal_node
    if not start or not goal:
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

    # SUMO entegrasyonu
    if not args.dry_run:
        try:
            adapter = SumoAdapter()
            if not adapter.connect(args.config, gui=args.gui):
                logger.warning("SUMO bağlantısı başarısız; sadece rota hesaplandı.")
                return 0

            replan_interval = float(getattr(args, 'replan_interval', 10.0))
            logger.info(f"SUMO bağlantısı kuruldu. {replan_interval:.0f} saniyede bir yeniden planlama çalışacak.")

            # TLSMap + Controller
            tl_ids = adapter.get_traffic_light_ids()
            main_tl = tl_ids[0] if tl_ids else None
            tls_map = _build_tls_map_auto(main_tl) if main_tl else None
            tlc = TrafficLightController(main_tl, tls_map=tls_map)

            # (Opsiyonel) ANFIS modelini TLS kontrolörüne tak
            if args.anfis_model and hasattr(tlc, "enable_anfis"):
                try:
                    tlc.enable_anfis(args.anfis_model)
                    logger.info(f"ANFIS modeli yüklendi: {args.anfis_model}")
                except Exception as e:
                    logger.warning(f"ANFIS yüklenemedi, kural tabanı kullanılacak: {e}")

            import random
            acc = 0.0
            loops = 0
            DEFAULT_HOSPITAL = "cluster_6762197026_6762197027_6762197028_6762197029"
            goal_node = goal or DEFAULT_HOSPITAL
            if goal_node not in router.nodes:
                cands = [nid for nid in router.nodes.keys() if nid.startswith('cluster_9855125')]
                goal_node = cands[0] if cands else list(router.nodes.keys())[-1]

            spawn_period = max(5.0, float(args.spawn_period))
            spawn_acc = 0.0
            last_spawn_sim_t = 0.0
            spawn_seq = 0

            # İlk ambulansı hemen üret
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

            max_loops = 1_000_000
            last_replan_sim_t = -1.0
            incr_search = None
            replan_in_flight = False

            # Preemption durum değişkenleri (WARN hata düzeltildi)
            last_preempt_tls_id: str | None = None
            preempt_until: float = -1.0

            while adapter.connected and loops < max_loops:
                adapter.step()

                # Kural tabanlı / ANFIS denetimi (CSV yazımı da burada yapılır)
                try:
                    if tlc and tlc.tls_map:
                        tlc.run_step()
                except Exception as e:
                    logger.warning(f"tlc.run_step() error: {e}")

                loops += 1
                acc += adapter.get_step_length_seconds()
                spawn_acc += adapter.get_step_length_seconds()

                cur_t = adapter.get_sim_time()

                # Periyodik ambulans üretimi
                if (cur_t - last_spawn_sim_t) >= spawn_period and cur_t > 0:
                    spawn_acc = 0.0
                    last_spawn_sim_t = cur_t
                    nodes_list = router.nodes_reaching(goal_node) or list(router.nodes.keys())
                    start_node_spawn = random.choice(nodes_list)
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

                if last_replan_sim_t < 0:
                    last_replan_sim_t = cur_t

                veh_ids = adapter.get_vehicle_ids()
                ambulance_id = next((v for v in veh_ids if any(k in adapter.get_vehicle_type(v).lower() for k in ("emergency", "ambulance"))), None)
                start_node_runtime = start
                if ambulance_id:
                    x, y = adapter.get_vehicle_position(ambulance_id)
                    snapped = router.nearest_node(x, y)
                    if snapped:
                        start_node_runtime = snapped

                # --- Ambulans yaklaştığı kavşakta acil öncelik ver + sonra restore et ---
                try:
                    if tlc and ambulance_id:
                        import traci
                        edge_now = traci.vehicle.getRoadID(ambulance_id)
                        nxt = traci.vehicle.getNextTLS(ambulance_id)  # [(tlsID, idx, dist, state), ...]
                        if nxt:
                            tls_id, _, dist, _ = nxt[0]
                            if dist is not None and dist < 80:  # mesafe eşiği
                                if last_preempt_tls_id and last_preempt_tls_id != tls_id:
                                    tlc.restore(last_preempt_tls_id)
                                if tlc.give_priority(tls_id, approach_edge_id=edge_now, green_seconds=10.0):
                                    last_preempt_tls_id = tls_id
                                    preempt_until = cur_t + 12.0  # 10 sn yeşil + 2 sn tampon
                        # Süre dolduysa restore et
                        if last_preempt_tls_id and cur_t >= preempt_until:
                            tlc.restore(last_preempt_tls_id)
                            last_preempt_tls_id = None
                except Exception as e:
                    logger.warning(f"priority preemption error: {e}")

                # Replan tetikle
                if (cur_t - last_replan_sim_t) >= replan_interval and cur_t > 0 and not replan_in_flight:
                    last_replan_sim_t = cur_t

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

                    edges_subset = collect_local_edges(start_node_runtime, max_depth=2, max_edges=200)
                    edge_stats_snapshot = adapter.get_edges_stats_subset(edges_subset) if edges_subset else {}
                    incr_search = IncrementalAStar(router, start_node_runtime, goal_node, edge_stats_snapshot)
                    replan_in_flight = True

                # Replan sonucu
                if incr_search is not None and replan_in_flight:
                    incr_search.step(max_expansions=50)
                    if incr_search.finished():
                        res_time, res_path = incr_search.get_result()
                        if res_time == float('inf') or not res_path:
                            incr_search = None
                            replan_in_flight = False
                        else:
                            best_time, best_path = res_time, res_path
                            edge_stats_used = incr_search.edge_stats
                            _ = best_time, best_path, edge_stats_used  # şimdilik sadece hesaplandı
                            incr_search = None
                            replan_in_flight = False

            adapter.close()

            # ---- Otomatik eğitim (opsiyonel) ----
            if args.auto_train:
                try:
                    csv_path = "data/tls_logs.csv"
                    out_path = "data/anfis.pt"
                    if not os.path.exists(csv_path):
                        logger.warning(f"Auto-train atlandı; CSV yok: {csv_path}")
                    else:
                        # train_anfis.py varsayılan yolları kullanıyorsa flags gerekmeyebilir.
                        subprocess.run(
                            [sys.executable, "-m", "src.scripts.train_anfis", "--csv", csv_path, "--out", out_path],
                            check=True
                        )
                        logger.info(f"ANFIS yeniden eğitildi ve kaydedildi: {out_path}")
                except Exception as e:
                    logger.warning(f"Auto-train başarısız: {e}")

        except Exception as e:
            logger.warning(f"SUMO entegrasyonu sırasında hata: {e}")

    return 0


# -------------------------------- Argparser -------------------------------- #
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
    run.add_argument("--anfis-model", default=None, help="Eğitimli ANFIS modeli (örn: data/anfis.pt)")
    run.add_argument("--auto-train", action="store_true", help="Koşudan sonra tls_logs.csv ile ANFIS'i yeniden eğit")
    run.set_defaults(func=cmd_run)

    return parser


def main():
    parser = build_arg_parser()
    args = parser.parse_args()
    return args.func(args)


if __name__ == "__main__":
    sys.exit(main())
