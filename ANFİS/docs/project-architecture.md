# Proje Mimarisi

## Sistem Genel Bakış

Bu sistem, SUMO üzerinde ambulanslar için rota (A* + ALT) üretir ve ANFIS tabanlı trafik ışığı önceliği uygular. Simülasyon akışı durmadan yeniden planlama yapılır.

## Temel Bileşenler

### 1. Ağ ve Konfigürasyon
- **Konum:** `config/` ve `maps/`
- **Dosyalar:**
  - `maps/map-new.osm` (örnek OSM)
  - `config/network_with_tl.net.xml` (SUMO ağ dosyası)
  - `config/simulation.sumocfg` (simülasyon)

### 2. Offline/Online Bileşenler
- **Konum:** `src/`
- **Ana Modüller:**
  - `src/online/router.py`: A* + ALT yönlendirme, ağ ayrıştırma
  - `src/offline/landmarks.py`: Landmark ön-hazırlık
  - `src/adapters/sumo_adapter.py`: SUMO/TraCI adaptörü
  - `src/controllers/traffic_light.py`: ANFIS tabanlı ışık önceliği
  - `src/ai/anfis.py`: ANFIS çıkarım (TriMF, kurallar, params)
  - `src/main.py`: Orkestratör (CLI, spawn, replan)

### 3. Eğitim ve Modeller
- **Konum:** `scripts/`, `models/`, `data/`
- `scripts/train_anfis.py`: Loglardan model üretir/günceller
- `models/anfis.json`: ANFIS model parametreleri
- `data/signal_training_v2.csv`: Eğitim logları (simülasyonca yazılır)

## Sistem Akış Diyagramı

```
GPS Verisi → Ambulans Konumu → Mesafe Hesaplama → Trafik Işığı Kontrolü
     ↑              ↓                ↓                   ↓
Simülasyon ← TraCI Interface ← Algılama Sistemi ← Işık Değişimi
```

## Teknik Detaylar

### Trafik Işığı Önceliği (ANFIS)
1. ANFIS `predict_trigger_prob` ile tetikleme olasılığı hesaplanır; `params.trigger_threshold` eşiği kullanılır.
2. Ambulans çok yakınsa `params.near_force_distance_m` ile zorunlu tetikleme uygulanır.
3. ANFIS `predict_extend_seconds` ile yeşil süresi belirlenir; `min_green`/`max_green` sınırlarıyla kırpılır.
4. Ambulans kavşağı geçip `params.release_distance_m` kadar uzaklaşana kadar yeşil korunur, sonra normale dönülür.

### Önemli Parametreler
- `models/anfis.json > params.trigger_threshold`
- `models/anfis.json > params.near_force_distance_m`
- `models/anfis.json > params.release_distance_m`

## Performans Gereksinimleri

- **Gerçek Zamanlı Simülasyon:** 1:1 zaman oranı
- **Yanıt Süresi:** Ambulans algılandıktan sonra maksimum 5 saniye
- **Simültane Ambulans:** En fazla 10 ambulans
- **Harita Büyüklüğü:** Burdur-Bucak merkezi (~50 km²)

## Genişletilebilirlik

### Gelecek Özellikler:
1. **Çoklu Acil Durum Araçları:** İtfaiye, polis
2. **Öncelik Sıralaması:** Ambulans vs. itfaiye
3. **Trafik Yoğunluğu Analizi:** Işık sürelerini optimize etme
4. **Gerçek GPS Entegrasyonu:** Canlı veri akışı
5. **Web Dashboard:** Simülasyonu izleme arayüzü

## Güvenlik ve Doğrulama

### Test Senaryoları:
1. Tek ambulans - tek trafik ışığı
2. Çoklu ambulans - aynı anda
3. Ambulans - yoğun trafik
4. Sistem arızası - failsafe modları

### Performans Metrikleri:
- Ambulans gecikme süresi
- Normal trafik üzerindeki etki
- Sistem yanıt süresi
- Enerji/yakıt tasarrufu 