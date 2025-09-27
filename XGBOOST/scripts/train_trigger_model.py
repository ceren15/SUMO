#!/usr/bin/env python3
"""
XGBoost model eğitimi: should_trigger_priority için
Bu model ambulansın trafik ışığına ne kadar yaklaştığında öncelik verilmesi gerektiğini öğrenir.
"""

import os
import sys
import pandas as pd
import numpy as np
import xgboost as xgb
from sklearn.model_selection import train_test_split
from sklearn.metrics import accuracy_score, classification_report
import json
import random

def generate_training_data(num_samples=5000):
    """Sentetik eğitim verisi oluştur"""
    print(f"Sentetik eğitim verisi oluşturuluyor ({num_samples} örnek)...")
    
    data = []
    
    for _ in range(num_samples):
        # Mesafe: 0-300m arası
        dist_to_tls = random.uniform(0, 300)
        
        # Ambulans hızı: 0-20 m/s (0-72 km/h)
        ambulance_speed = random.uniform(0, 20)
        
        # Kuyruk boyu: 0-100m arası
        queue_length = random.uniform(0, 100)
        
        # ETA hesapla
        eta_seconds = (dist_to_tls / max(0.5, ambulance_speed)) if ambulance_speed > 0.01 else 9999.0
        
        # Faz bilgileri
        phase_index = random.randint(0, 7)
        phase_remaining = random.uniform(0, 30)
        
        # Hedef değişken: tetiklenmeli mi? (0: hayır, 1: evet)
        # Kural tabanlı hedef oluştur
        should_trigger = 0
        
        # Mesafe çok yakınsa (50m altı) her zaman tetikle
        if dist_to_tls <= 50:
            should_trigger = 1
        # ETA çok kısaysa (5s altı) her zaman tetikle
        elif eta_seconds <= 5:
            should_trigger = 1
        # Hızlı ambulans + uzun kuyruk = erken tetikle
        elif ambulance_speed > 10 and queue_length > 50:
            if dist_to_tls <= 120:
                should_trigger = 1
        # Orta hız + orta kuyruk
        elif ambulance_speed > 5 and queue_length > 20:
            if dist_to_tls <= 100:
                should_trigger = 1
        # Yavaş ambulans + kısa kuyruk = geç tetikle
        elif ambulance_speed < 5 and queue_length < 20:
            if dist_to_tls <= 60:
                should_trigger = 1
        # Faz değişimi yakınsa daha erken tetikle
        elif phase_remaining < 5:
            if dist_to_tls <= 80:
                should_trigger = 1
        
        # Rastgele gürültü ekle (%10)
        if random.random() < 0.1:
            should_trigger = 1 - should_trigger
        
        data.append({
            'dist_to_tls': dist_to_tls,
            'ambulance_speed': ambulance_speed,
            'queue_length': queue_length,
            'eta_seconds': eta_seconds,
            'phase_index': phase_index,
            'phase_remaining': phase_remaining,
            'should_trigger': should_trigger
        })
    
    return pd.DataFrame(data)

def train_model(df):
    """XGBoost modeli eğit"""
    print("Model eğitimi başlıyor...")
    
    # Özellikler ve hedef
    feature_cols = ['dist_to_tls', 'ambulance_speed', 'queue_length', 'eta_seconds', 'phase_index', 'phase_remaining']
    X = df[feature_cols]
    y = df['should_trigger']
    
    # Train/test split
    X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.2, random_state=42, stratify=y)
    
    # XGBoost parametreleri
    params = {
        'objective': 'binary:logistic',
        'eval_metric': 'logloss',
        'max_depth': 6,
        'learning_rate': 0.1,
        'n_estimators': 100,
        'subsample': 0.8,
        'colsample_bytree': 0.8,
        'random_state': 42
    }
    
    # Model eğit
    model = xgb.XGBClassifier(**params)
    model.fit(X_train, y_train)
    
    # Test
    y_pred = model.predict(X_test)
    accuracy = accuracy_score(y_test, y_pred)
    
    print(f"Test Accuracy: {accuracy:.4f}")
    print("\nClassification Report:")
    print(classification_report(y_test, y_pred))
    
    # Özellik önemleri
    feature_importance = pd.DataFrame({
        'feature': feature_cols,
        'importance': model.feature_importances_
    }).sort_values('importance', ascending=False)
    
    print("\nFeature Importance:")
    print(feature_importance)
    
    return model

def save_model(model, output_path):
    """Modeli kaydet"""
    print(f"Model kaydediliyor: {output_path}")
    
    # JSON formatında kaydet
    model.save_model(output_path)
    
    # Model bilgilerini kaydet
    model_info = {
        'feature_names': ['dist_to_tls', 'ambulance_speed', 'queue_length', 'eta_seconds', 'phase_index', 'phase_remaining'],
        'feature_types': ['float', 'float', 'float', 'float', 'int', 'float'],
        'model_type': 'binary_classification',
        'threshold': 0.5,
        'description': 'Ambulans trafik ışığı öncelik tetikleme modeli'
    }
    
    info_path = output_path.replace('.json', '_info.json')
    with open(info_path, 'w') as f:
        json.dump(model_info, f, indent=2)
    
    print(f"Model bilgileri kaydedildi: {info_path}")

def main():
    """Ana fonksiyon"""
    print("=== XGBoost Trigger Model Eğitimi ===")
    
    # Çıktı dizinini oluştur
    os.makedirs('models', exist_ok=True)
    os.makedirs('data', exist_ok=True)
    
    # Eğitim verisi oluştur
    df = generate_training_data(num_samples=10000)
    
    # Veriyi kaydet
    df.to_csv('data/trigger_training.csv', index=False)
    print(f"Eğitim verisi kaydedildi: data/trigger_training.csv")
    print(f"Veri dağılımı: {df['should_trigger'].value_counts().to_dict()}")
    
    # Model eğit
    model = train_model(df)
    
    # Modeli kaydet
    save_model(model, 'models/trigger_xgb.json')
    
    print("\n=== Eğitim Tamamlandı ===")
    print("Model dosyası: models/trigger_xgb.json")
    print("Model bilgileri: models/trigger_xgb_info.json")
    print("Eğitim verisi: data/trigger_training.csv")

if __name__ == "__main__":
    main()

