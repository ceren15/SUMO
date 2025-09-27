#!/usr/bin/env python3
"""
ANFIS parametre eğitimi:

Girdi: data/signal_training.csv (feats + y_extend) ve/veya data/dir_training.csv (label_select)
Çıktı: models/anfis.json (fuzzy_sets ve kuralların ağırlıkları)

Not: Bu minimal bir optimizasyon örneğidir. Üyelik fonksiyonlarının merkezlerini ve
kuralların ağırlıklarını basitçe ayarlar. Gelişmiş eğitim için gradyan temelli
yaklaşımlar veya ANFIS spesifik kütüphaneler tercih edilebilir.
"""

import os
import json
import math
import random
from typing import Dict, Any, List

import pandas as pd


DEFAULT_MODEL = {
	"min_green": 6.0,
	"max_green": 20.0,
	"fuzzy_sets": {
		"dist_to_tls": {"near": [0, 30, 80], "mid": [50, 120, 200], "far": [150, 300, 500]},
		"ambulance_speed": {"low": [0, 2, 5], "med": [3, 7, 11], "high": [9, 14, 20]},
		"queue_length": {"short": [0, 0, 10], "med": [5, 20, 40], "long": [30, 60, 100]},
		"eta_seconds": {"soon": [0, 4, 8], "mid": [6, 10, 16], "late": [12, 20, 35]},
		"phase_remaining": {"short": [0, 1, 3], "mid": [2, 6, 10], "long": [8, 14, 22]}
	},
	"rules_trigger": [
		{"if": {"dist_to_tls": "near", "eta_seconds": "soon"}, "w": 1.0},
		{"if": {"queue_length": "long"}, "w": 0.7}
	],
	"rules_extend": [
		{"if": {"dist_to_tls": "near"}, "w": 10.0},
		{"if": {"queue_length": "long"}, "w": 4.0}
	],
	"params": {
		"trigger_threshold": 0.5,
		"near_force_distance_m": 200.0,
		"release_distance_m": 50.0
	}
}


def load_training() -> Dict[str, pd.DataFrame]:
	frames: Dict[str, pd.DataFrame] = {}
	# Prefer new schema if exists
	if os.path.exists("data/signal_training_v2.csv"):
		frames["signal"] = pd.read_csv("data/signal_training_v2.csv")
	elif os.path.exists("data/signal_training.csv"):
		frames["signal"] = pd.read_csv("data/signal_training.csv")
	if os.path.exists("data/dir_training.csv"):
		frames["dir"] = pd.read_csv("data/dir_training.csv")
	return frames


def train_model(frames: Dict[str, pd.DataFrame]) -> Dict[str, Any]:
	model = json.loads(json.dumps(DEFAULT_MODEL))
	# Basit ayarlama: extend hedeflerinin medyanına göre dist_to_tls near üçgen merkezini güncelle
	sig = frames.get("signal")
	if sig is not None and not sig.empty:
		try:
			med_extend = float(sig["y_extend"].median())
			# medyan 6-20 arasında olmalı; dist_to_tls near merkezini (b) 30→(20..60) arasında ayarla
			b = max(20.0, min(60.0, 80.0 - (med_extend - 6.0) * 2.0))
			model["fuzzy_sets"]["dist_to_tls"]["near"][1] = b
		except Exception:
			pass
	# Kural ağırlık basitleştirmesi: kuyruk büyükse daha fazla uzatma
	if sig is not None and not sig.empty:
		try:
			ql = sig.get("queue_length")
			ext = sig.get("y_extend")
			if ql is not None and ext is not None:
				corr = float(pd.Series(ql).corr(pd.Series(ext)))
				# pozitif korelasyon varsa long kuralını kuvvetlendir
				model["rules_extend"][1]["w"] = float(max(2.0, min(8.0, 4.0 + corr * 2.0)))
		except Exception:
			pass
	# Eşik parametreleri (isteğe bağlı basit kalibrasyon)
	model.setdefault("params", {})
	params = model["params"]
	params.setdefault("trigger_threshold", 0.5)
	params.setdefault("near_force_distance_m", 200.0)
	params.setdefault("release_distance_m", 50.0)
	return model


def main() -> int:
	frames = load_training()
	if not frames:
		print("No training data found under data/. Run the simulation to accumulate logs.")
		return 1
	model = train_model(frames)
	os.makedirs("models", exist_ok=True)
	with open("models/anfis.json", "w", encoding="utf-8") as f:
		json.dump(model, f, ensure_ascii=False, indent=2)
	print("Saved models/anfis.json")
	return 0


if __name__ == "__main__":
	raise SystemExit(main())


