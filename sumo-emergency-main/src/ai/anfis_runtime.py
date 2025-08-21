#!/usr/bin/env python3
import torch
from src.ai.anfis_nn import SugenoANFIS

FEATURES = ["queue_main","queue_side","wait_main","wait_side",
            "on_main_green","green_timer","ambu_main","ambu_side"]

class ANFISRuntime:
    def __init__(self, model_path: str, extend_clip=(0.0,10.0)):
        self.model = SugenoANFIS(in_dim=len(FEATURES), terms_per_feat=3, out_dim=2)
        state = torch.load(model_path, map_location="cpu")
        self.model.load_state_dict(state["state_dict"])
        self.model.eval()
        self.min_ext, self.max_ext = extend_clip

    def decide(self, features: dict, min_green: int, green_timer: int) -> dict:
        x = torch.tensor([
            float(features.get("queue_main",0))/20.0,
            float(features.get("queue_side",0))/20.0,
            float(features.get("wait_main",0))/60.0,
            float(features.get("wait_side",0))/60.0,
            float(features.get("on_main_green",0)),
            float(green_timer)/40.0,
            float(features.get("ambu_main",0)),
            float(features.get("ambu_side",0)),
        ], dtype=torch.float32).unsqueeze(0)
        with torch.no_grad():
            y = self.model(x)[0]
        extend = float(y[0])
        switch_p = float(y[1])
        force_switch = (switch_p > 0.5) and (green_timer >= min_green)
        extend = max(self.min_ext, min(self.max_ext, extend))
        return {"extend": extend, "force_switch": force_switch}
