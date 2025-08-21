#!/usr/bin/env python3
import csv, os, torch
import torch.nn as nn
from torch.utils.data import Dataset, DataLoader
from src.ai.anfis_nn import SugenoANFIS

FEATURES = ["queue_main","queue_side","wait_main","wait_side",
            "on_main_green","green_timer","ambu_main","ambu_side"]

class TLSDataset(Dataset):
    def __init__(self, csv_path):
        self.rows = []
        with open(csv_path, newline="", encoding="utf-8") as f:
            for r in csv.DictReader(f):
                self.rows.append(r)
    def __len__(self): return len(self.rows)
    def __getitem__(self, i):
        r = self.rows[i]
        x = torch.tensor([
            float(r["queue_main"])/20.0,
            float(r["queue_side"])/20.0,
            float(r["wait_main"])/60.0,
            float(r["wait_side"])/60.0,
            float(r["on_main_green"]),
            float(r.get("green_timer", 0))/40.0,
            float(r["ambu_main"]),
            float(r["ambu_side"]),
        ], dtype=torch.float32)
        y = torch.tensor([
            float(r["extend_seconds"]),
            float(r["switch_flag"])
        ], dtype=torch.float32)
        return x, y

def train(csv_path="data/tls_logs.csv", out_path="data/anfis.pt",
          epochs=15, batch=128, lr=1e-3):
    ds = TLSDataset(csv_path)
    if len(ds) < 100:
        raise RuntimeError(f"yetersiz veri: {len(ds)} satır (min 100 önerilir)")
    dl = DataLoader(ds, batch_size=batch, shuffle=True)
    model = SugenoANFIS(in_dim=len(FEATURES), terms_per_feat=3, out_dim=2)
    opt = torch.optim.Adam(model.parameters(), lr=lr)
    mse = nn.MSELoss(); bce = nn.BCELoss()

    for ep in range(epochs):
        model.train(); total=0.0
        for x,y in dl:
            y_hat = model(x)
            loss = mse(y_hat[:,0], y[:,0]) + bce(y_hat[:,1], y[:,1])
            opt.zero_grad(); loss.backward(); opt.step()
            total += loss.item()*x.size(0)
        print(f"epoch {ep+1}/{epochs}  loss={total/len(ds):.4f}")

    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    torch.save({"state_dict": model.state_dict()}, out_path)
    print(f"saved -> {out_path}")

if __name__ == "__main__":
    train()
