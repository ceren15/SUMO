#!/usr/bin/env python3
"""
ANFIS çıkarım motoru (Sugeno-tipi) – sinyal öncelik tetiklemesi ve yeşil uzatma.

Not: Bu sürüm üretim için minimaldir. Bir model dosyası (.json) verilirse üyelik
fonksiyonları ve kural ağırlıklarını yükler. Aksi halde mantıklı varsayılan
üçgensel üyelikler ve birkaç kural ile çalışır.
"""

from typing import Dict, Any, Optional, List, Tuple
import json
import math


class TriMF:
	"""Üçgensel üyelik fonksiyonu: (a, b, c)."""
	def __init__(self, a: float, b: float, c: float):
		self.a = float(a)
		self.b = float(b)
		self.c = float(c)

	def mu(self, x: float) -> float:
		x = float(x)
		if x <= self.a or x >= self.c:
			return 0.0
		if x == self.b:
			return 1.0
		if x < self.b:
			return max(0.0, (x - self.a) / max(1e-6, (self.b - self.a)))
		return max(0.0, (self.c - x) / max(1e-6, (self.c - self.b)))


def clamp(v: float, lo: float, hi: float) -> float:
	return max(lo, min(hi, v))


class AnfisModel:
	"""ANFIS modeli.

	Girişler: dist_to_tls (m), ambulance_speed (m/s), queue_length (m),
	          eta_seconds (s), phase_index, phase_remaining (s)

	Çıkış 1 (tetikleme olasılığı): [0,1]
	Çıkış 2 (yeşil uzatma süresi): [min_sec, max_sec]
	"""

	def __init__(self, model_path: Optional[str] = None):
		self.model_path = model_path
		self.loaded = False
		self.fuzzy_sets: Dict[str, Dict[str, TriMF]] = {}
		self.rules_trigger: List[Tuple[Dict[str, str], float]] = []
		self.rules_extend: List[Tuple[Dict[str, str], float]] = []
		self.min_green = 6.0
		self.max_green = 20.0
		self._init_default()
		if model_path:
			self._try_load(model_path)
		# Ek model parametreleri (eşikler, histerezis)
		self.params: Dict[str, float] = {
			"trigger_threshold": 0.5,
			"near_force_distance_m": 200.0,
			"release_distance_m": 50.0,
			"min_green": self.min_green,
			"max_green": self.max_green,
		}

	def _init_default(self) -> None:
		# Üyelik fonksiyonları (hedefe göre kaba değerler)
		self.fuzzy_sets = {
			"dist_to_tls": {
				"near": TriMF(0.0, 30.0, 80.0),
				"mid": TriMF(50.0, 120.0, 200.0),
				"far": TriMF(150.0, 300.0, 500.0),
			},
			"ambulance_speed": {
				"low": TriMF(0.0, 2.0, 5.0),
				"med": TriMF(3.0, 7.0, 11.0),
				"high": TriMF(9.0, 14.0, 20.0),
			},
			"queue_length": {
				"short": TriMF(0.0, 0.0, 10.0),
				"med": TriMF(5.0, 20.0, 40.0),
				"long": TriMF(30.0, 60.0, 100.0),
			},
			"eta_seconds": {
				"soon": TriMF(0.0, 4.0, 8.0),
				"mid": TriMF(6.0, 10.0, 16.0),
				"late": TriMF(12.0, 20.0, 35.0),
			},
			"phase_remaining": {
				"short": TriMF(0.0, 1.0, 3.0),
				"mid": TriMF(2.0, 6.0, 10.0),
				"long": TriMF(8.0, 14.0, 22.0),
			},
		}
		# Kurallar: (koşullar -> ağırlık). Ağırlıklar 0..1 arasında.
		self.rules_trigger = [
			({"dist_to_tls": "near", "eta_seconds": "soon"}, 1.0),
			({"dist_to_tls": "near", "queue_length": "long"}, 0.9),
			({"dist_to_tls": "mid", "ambulance_speed": "high"}, 0.8),
			({"queue_length": "long"}, 0.7),
			({"phase_remaining": "short", "eta_seconds": "soon"}, 0.85),
		]
		# Uzatma: ağırlık, saniyeye ölçeklenir
		self.rules_extend = [
			({"dist_to_tls": "near"}, 10.0),
			({"queue_length": "long"}, 4.0),
			({"ambulance_speed": "low"}, 2.0),
			({"phase_remaining": "short"}, 3.0),
		]

	def _try_load(self, path: str) -> None:
		try:
			with open(path, "r", encoding="utf-8") as f:
				data = json.load(f)
			# Fuzzy sets
			fs = {}
			for var, mfs in data.get("fuzzy_sets", {}).items():
				fs[var] = {}
				for name, params in mfs.items():
					fs[var][name] = TriMF(*params)
			self.fuzzy_sets = fs or self.fuzzy_sets
			# Rules
			self.rules_trigger = [(r["if"], float(r.get("w", 1.0))) for r in data.get("rules_trigger", [])] or self.rules_trigger
			self.rules_extend = [(r["if"], float(r.get("w", 1.0))) for r in data.get("rules_extend", [])] or self.rules_extend
			self.min_green = float(data.get("min_green", self.min_green))
			self.max_green = float(data.get("max_green", self.max_green))
			# Opsiyonel parametreler
			params = data.get("params", {})
			if isinstance(params, dict):
				for k, v in params.items():
					try:
						self.params[k] = float(v)
					except Exception:
						pass
			self.loaded = True
		except Exception:
			self.loaded = False

	def _mu(self, var: str, label: str, x: float) -> float:
		try:
			return self.fuzzy_sets[var][label].mu(x)
		except Exception:
			return 0.0

	def _rule_fire(self, cond: Dict[str, str], feats: Dict[str, float]) -> float:
		f = 1.0
		for var, label in cond.items():
			x = float(feats.get(var, 0.0))
			f = min(f, self._mu(var, label, x))
		return f

	def predict_trigger_prob(self, feats: Dict[str, float]) -> float:
		# Basit ağırlıklı maksimum (Sugeno benzeri):
		# prob = max_i( fire_i * w_i )
		best = 0.0
		for cond, w in self.rules_trigger:
			fire = self._rule_fire(cond, feats)
			best = max(best, fire * clamp(w, 0.0, 1.0))
		return clamp(best, 0.0, 1.0)

	def predict_extend_seconds(self, feats: Dict[str, float]) -> float:
		# Uzatma saniyesi: taban 6s + katkılar (kural ateşleme * w)
		sec = self.min_green
		for cond, w in self.rules_extend:
			fire = self._rule_fire(cond, feats)
			sec += max(0.0, fire * w)
		return clamp(sec, self.min_green, self.max_green)
