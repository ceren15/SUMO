## SUMO Emergency — Akıllı Ambulans Rota ve Işık Yönetimi

### Genel Bakış
SUMO üzerinde gerçek zamanlı ambulans yönlendirme ve trafik ışığı önceliği. Sistem A* + ALT (landmark) ile rota çıkarır, canlı trafik verileriyle kenar maliyetlerini ayarlar ve simülasyonu durdurmadan sinyal önceliği uygular. Ambulanslar periyodik olarak üretilir ve rotaları sürekli yeniden değerlendirilir.

### Özellikler
- ALT (landmark) sezgisiyle çevrimiçi A* yönlendirme
- Canlı trafik etkili kenar maliyetleri (araç sayısı + anlık hız)
- Simülasyonu durdurmadan, artımlı (incremental) yeniden planlama
- ANFIS tabanlı trafik ışığı önceliği (tetikleme + yeşil uzatma)
- Otomatik ambulans üretimi (ilki anında; sonrası periyodik)
- CLI ile yapılandırılabilir (hedef düğüm, spawn periyodu, replan aralığı)

## Gereksinimler
- SUMO (sumo-gui ve TraCI) kurulu ve PATH’te
- Python 3.10+

## Proje Yapısı
- `src/main.py`: Orkestratör (CLI, döngü, spawn, replan, loglar)
- `src/online/router.py`: Çevrimiçi A* (ALT, ağ ayrıştırma, yardımcılar)
- `src/adapters/sumo_adapter.py`: SUMO/TraCI adaptörü (step, araç/rota ekleme, kenar istatistikleri)
- `config/simulation.sumocfg`: SUMO simülasyon yapılandırması
- `config/network_with_tl.net.xml`: Otomatik tahmin edilmiş trafik ışıklarıyla ağ
- `data/landmarks.json`: ALT için landmark tabloları

## Kurulum ve Çalıştırma
1) Landmark üret (ALT tabloları):
```bash
python -m src.main prep-landmarks --net config/network_with_tl.net.xml --output data/landmarks.json
```

2) Simülasyonu başlat (GUI):
```bash
python -m src.main run --config config/simulation.sumocfg --gui --replan-interval 10 --spawn-period 60
```
- SUMO-GUI’de Play’e basarak simülasyon zamanını başlat.

## Canlı Yeniden Yönlendirme (Nasıl çalışır?)
Yeniden planlama periyodik olarak tetiklenir (varsayılan 10 sn) ve artımlı çalışır; simülasyon adımları akmaya devam eder.

### Akış
1. Ambulansın en yakın ağ düğümü bulunur (snap-to-node)
2. Tüm ağ yerine ambulans çevresindeki küçük bir altgraf için canlı trafik metrikleri toplanır (2 sıçrama, en fazla 200 edge)
3. Artımlı A* başlatılır; her simülasyon adımında en fazla 50 düğüm genişletilerek (non-blocking) arama ilerletilir
4. A* bittiğinde karar loglanır ve yaklaşan koridora sinyal önceliği verilir

### Maliyet modeli (g/h)
- Kenar taban süresi: ort. şerit uzunluğu / serbest akış hızı
- Canlı katsayı: sıkışıklık (≈ v_ref / v) ve yük (≈ 1 + veh/20) birleşimi; [1.0, 5.0] aralığına kırpılır
- Sinyal gecikmesi: kanca mevcut (`get_signal_delay`), varsayılan 0.0 (ANFIS’e hazır)
- Heuristik: ALT (landmark); ANFIS düzeltme kancası konservatif bırakılmıştır

## Loglar (Neye bakmalıyım?)
- Başlangıç/rota önizleme:
  - `A* rota hesaplanıyor: start=... → goal=...`
  - `Rota bulundu. Süre (tahmini): ... s, Düğümler: ...`
- SUMO bağlantısı ve periyot:
  - `SUMO bağlantısı kuruldu. 10 saniyede bir yeniden planlama çalışacak.`
- Spawn:
  - `İlk ambulans: ambulance_0, from=... → ..., edges=N`
  - `Yeni ambulans: ambulance_k, from=... → ..., edges=N`
- Yeniden planlama (~X saniyede bir):
  - `[Edges] edge1: base=...s live=... adj=...s | edge2: ... | edge3: ...`
  - `[Replan] t=...s ETA~...s, düğüm: ...`
  - `[ALT-KIYAS] edgeA=... t~Xs vs edgeB=... t~Ys → seçilen=...` (opsiyonel koridor kıyası)

Yorumlama:
- `live` 1.00’dan sapıyorsa canlı trafik kararları etkiliyor demektir
- ETA veya düğüm sayısındaki değişimler replan’ın aktif olduğuna işaret eder

## CLI Parametreleri
- `--config`: SUMO `.sumocfg` yolu (vars: `config/simulation.sumocfg`)
- `--gui`: SUMO-GUI ile çalıştır
- `--goal-node`: Hastane junction ID (vars: `cluster_6762197026_6762197027_6762197028_6762197029`)
- `--spawn-period`: Periyodik ambulans üretim aralığı (s) (vars: `60.0`)
- `--replan-interval`: Yeniden planlama aralığı (s) (vars: `10.0`)
- `--anfis-model`: ANFIS model dosyası (vars: `models/anfis.json`)

Örnekler:
```bash
# Landmark üret
python -m src.main prep-landmarks --net config/network_with_tl.net.xml

# GUI ile çalıştır, 10 sn replan, varsayılan spawn periyodu
python -m src.main run --config config/simulation.sumocfg --gui --replan-interval 10

# Test için daha hızlı
python -m src.main run --config config/simulation.sumocfg --gui --replan-interval 6 --spawn-period 30
```

## Varsayılan Hedef ve Spawn
- Varsayılan hedef (hastane): `cluster_6762197026_6762197027_6762197028_6762197029` (`--goal-node` ile değiştirilebilir)
- İlk ambulans: GUI bağlanır bağlanmaz oluşturulur ve hastaneye rota alır
- Periyodik ambulanslar: Her `spawn_period` simülasyon saniyesinde, hedefe ulaşabilen rastgele düğümlerden

## Mimari Notlar
- `src/main.py` (cmd_run):
  - SUMO bağlantısı ve ana döngü
  - Ambulans spawn (ilki anında; sonrası periyodik)
  - `--replan-interval` ile artımlı A* tetikleme
  - Kenar kalemleri/ETA loglama; ANFIS tabanlı sinyal önceliği
- `src/online/router.py`:
  - Ağ (`.net.xml`) ayrıştırma, grafik kurma, taban süreler
  - `data/landmarks.json` ile ALT heuristik
  - Yardımcılar: `nearest_node`, `nodes_reaching`, `endpoints_to_edge`
- `src/adapters/sumo_adapter.py`:
  - `connect`, `simulationStep`, `get_sim_time`
  - `add_route`, `add_vehicle`, `set_route` (mevcut)
  - Canlı kenar istatistikleri: `get_edge_stats`, ve kapsamlı sürüm `get_edges_stats_subset(edges)`
 - `src/controllers/traffic_light.py`: ANFIS kararlarıyla trafik ışığı önceliği
 - `src/ai/anfis.py`: ANFIS çıkarımı (TriMF, kurallar, `params`)
 - `scripts/train_anfis.py`: Loglardan `models/anfis.json` üretimi/güncellemesi

## ANFIS Modeli ve Eğitim
- Varsayılan model yolu: `models/anfis.json` (otomatik yüklenir)
- Eğitim verisi: `data/signal_training_v2.csv` (simülasyon sırasında otomatik yazılır)
- Eğitimi çalıştır:
```bash
python scripts/train-anfis.py
```
- Çıktı: `models/anfis.json` (fuzzy_sets, rules, params: `trigger_threshold`, `near_force_distance_m`, `release_distance_m`)

## Sorun Giderme
- Ambulans görünmüyor: GUI’de Play’e bas. Spawn, simülasyon zamanına bağlıdır
- Tek ambulans: Simülasyon süresini uzat veya `--spawn-period` değerini küçült
- Replan log yok: `--replan-interval` ayarlı mı, GUI akıyor mu, döngü çalışıyor mu kontrol et
- `live` hep 1.00: Trafik düşük; arka plan araç üretimini artır veya daha uzun çalıştır

## Yol Haritası
- Hesaplanan rotayı canlı araca uygulama (dinamik `setRoute`) + histerezis
- Trafik ışığı önceliği: yaklaşım→faz eşleme, güvenlik kuralları, yeşil dalga
- ANFIS: heuristik ayarı ve sinyal gecikmesi tahmini + eğitim hattı
- Kalıcı loglar (JSONL) ve Prometheus metrikleri
- Senaryo profilleri (YAML) ve birim/entegrasyon testleri

---
Sorular için önce loglara bak (`[Edges]`, `[Replan]`, `[ALT-KIYAS]`) ve GUI’nin adım attığından emin ol. Sistem, yeniden planlama yaparken simülasyonun akıcı kalması için tasarlanmıştır.

