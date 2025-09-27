# Uygulama Rehberi (ANFIS Tabanlı)

Bu rehber, mevcut projeyi ANFIS tabanlı sinyal önceliği ile çalıştırmak ve eğitmek için gerekli adımları özetler.

## 0) Önkoşullar
- Python 3.10+
- SUMO (sumo, sumo-gui, TraCI) kurulu ve PATH’te
- Proje kökünde bağımlılıklar: `pip install -r requirements.txt`

## 1) Ağ Hazırlığı
Depoda örnek SUMO ağı ve simülasyon dosyaları sağlanmıştır:
- `config/network_with_tl.net.xml`
- `config/simulation.sumocfg`

Kendi OSM dosyanızdan ağ üretmek isterseniz netconvert kullanabilirsiniz:
```bash
netconvert --osm-files maps/your.osm \
           --output-file config/network_with_tl.net.xml \
           --geometry.remove --ramps.guess --junctions.join \
           --tls.guess-signals --tls.discard-simple
```

## 2) Landmark Tabloları (ALT)
Rota için ALT (landmark) sezgisini hazırlayın:
```bash
python -m src.main prep-landmarks \
  --net config/network_with_tl.net.xml \
  --output data/landmarks.json
```

## 3) Simülasyonu Çalıştırma (GUI ile)
```bash
python -m src.main run --config config/simulation.sumocfg --gui
```
Ne olur?
- A* + ALT ile rota hesaplanır, periyodik artımlı yeniden planlama yapılır.
- ANFIS tabanlı trafik ışığı önceliği: tetikleme ve yeşil süresi ANFIS tarafından belirlenir.
- Yeşil, ambulans kavşağı geçip yaklaşık `params.release_distance_m` (vars: 50 m) uzaklaşana kadar korunur.
- Eğitim verileri `data/signal_training_v2.csv` dosyasına yazılır.

İpuçları:
- Varsayılan ANFIS modeli otomatik yüklenir: `models/anfis.json`.
- İsterseniz `--replan-interval` ve `--spawn-period` ile akışı ayarlayabilirsiniz.

## 4) ANFIS Eğitimi
Simülasyon bir miktar veri ürettikten sonra modeli güncelleyin:
```bash
python scripts/train-anfis.py
```
Çıktı: `models/anfis.json` güncellenir (fuzzy_sets, rules ve `params`).

Önemli `params` anahtarları:
- `trigger_threshold`: Tetikleme eşiği (olasılık)
- `near_force_distance_m`: Yakınken zorla tetikleme mesafesi
- `release_distance_m`: Yeşili bırakma mesafesi

Güncel modeli kullanmak için simülasyonu tekrar çalıştırın.

## 5) Loglar ve Veriler
- Eğitim CSV: `data/signal_training_v2.csv` (simülasyon otomatik yazar)
- Landmark: `data/landmarks.json`
- Genel loglar: `logs/` (konsol INFO çıktıları önerilir)

## 6) Sık Kullanılan Komutlar
- Landmark üretme:
```bash
python -m src.main prep-landmarks --net config/network_with_tl.net.xml --output data/landmarks.json
```
- GUI ile çalıştırma:
```bash
python -m src.main run --config config/simulation.sumocfg --gui
```
- ANFIS eğitimi:
```bash
python scripts/train-anfis.py
```

## 7) Dosya ve Modül Haritası
- `src/main.py`: Orkestratör (CLI, spawn, replan, TL kontrol)
- `src/online/router.py`: A* + ALT yönlendirme
- `src/controllers/traffic_light.py`: ANFIS ile öncelik (tetikleme + yeşil)
- `src/ai/anfis.py`: ANFIS çıkarım motoru (TriMF, kurallar, `params`)
- `src/offline/landmarks.py`: Landmark ön-hazırlık
- `scripts/train-anfis.py`: Eğitim ve `models/anfis.json` yazımı

## 8) Parametre Ayarı (Hızlı)
`models/anfis.json` içindeki `params` bölümünden davranışı ince ayar yapabilirsiniz. Örneğin:
```json
{
  "params": {
    "trigger_threshold": 0.5,
    "near_force_distance_m": 200.0,
    "release_distance_m": 50.0
  }
}
```
Değişiklikten sonra simülasyonu yeniden başlatın. 