#!/usr/bin/env python3
import torch
import torch.nn as nn
import torch.nn.functional as F

class GaussianMF(nn.Module):
    def __init__(self, in_dim: int, terms_per_feat: int = 3):
        super().__init__()
        self.in_dim = in_dim
        self.terms = terms_per_feat
        self.mu = nn.Parameter(torch.randn(in_dim, terms_per_feat))
        self.log_sigma = nn.Parameter(torch.zeros(in_dim, terms_per_feat))

    def forward(self, x):  # x: [B,D]
        sigma = torch.exp(self.log_sigma) + 1e-3
        x_exp = x.unsqueeze(-1)                 # [B,D,1]
        gauss = torch.exp(-0.5 * ((x_exp - self.mu) / sigma)**2)
        return gauss.clamp(1e-6, 1.0)          # [B,D,K]

class SugenoANFIS(nn.Module):
    def __init__(self, in_dim: int, terms_per_feat: int = 3, out_dim: int = 2):
        super().__init__()
        self.in_dim = in_dim
        self.terms = terms_per_feat
        self.mf = GaussianMF(in_dim, terms_per_feat)
        self.num_rules = 64
        self.rule_selector = nn.Linear(in_dim, self.num_rules, bias=True)
        self.conseq_A = nn.Parameter(torch.zeros(self.num_rules, in_dim, out_dim))
        self.conseq_b = nn.Parameter(torch.zeros(self.num_rules, out_dim))

    def forward(self, x):  # x: [B,D]
        B, D = x.shape
        mf = self.mf(x)                     # [B,D,K]
        mf_max = mf.max(dim=2).values       # [B,D]
        z = mf_max                          # [B,D]
        w_logits = self.rule_selector(z)    # [B,R]
        w = F.softmax(w_logits, dim=1)      # [B,R]
        y_lin = torch.einsum("brd,rdo->bro", x.unsqueeze(1).expand(-1,self.num_rules,-1), self.conseq_A)
        y_lin = y_lin + self.conseq_b.unsqueeze(0)    # [B,R,O]
        y = torch.einsum("br,bro->bo", w, y_lin)      # [B,O]
        extend = torch.relu(y[:,0:1])
        switch_score = torch.sigmoid(y[:,1:2])
        return torch.cat([extend, switch_score], dim=1)
