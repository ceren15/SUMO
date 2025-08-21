# Proje Mimarisi

## Sistem Genel Bakış

Bu sistem, acil durum araçlarının trafik ışıklarını dinamik olarak kontrol etmesini sağlayan bir SUMO simülasyonudur.

## Temel Bileşenler

### 1. Harita ve Ağ Yapısı (Network)
- **Konum:** `maps/` klasörü
- **Dosyalar:**
  - `burdur-bucak.osm` - OpenStreetMap ham verisi
  - `burdur-bucak.net.xml` - SUMO ağ dosyası
  - `burdur-bucak.tll.xml` - Trafik ışıkları tanımlaması

### 2. Araç Tanımlamaları
- **Konum:** `config/` klasörü
- **Dosyalar:**
  - `vehicles.xml` - Araç türleri (ambulans, normal araçlar)
  - `routes.xml` - Araç rotaları
  - `emergency_vehicles.xml` - Acil durum araçları özel tanımlamaları

### 3. Simülasyon Kontrol Sistemi
- **Konum:** `src/` klasörü
- **Ana Modüller:**
  - `main.py` - Ana simülasyon kontrol
  - `traffic_light_controller.py` - Trafik ışığı kontrolü
  - `emergency_vehicle_detector.py` - Acil durum araçları algılaması
  - `gps_simulator.py` - GPS verilerini simüle etme

## Sistem Akış Diyagramı

```
GPS Verisi → Ambulans Konumu → Mesafe Hesaplama → Trafik Işığı Kontrolü
     ↑              ↓                ↓                   ↓
Simülasyon ← TraCI Interface ← Algılama Sistemi ← Işık Değişimi
```

## Teknik Detaylar

### Trafik Işığı Kontrol Algoritması

1. **Ambulans Algılama:**
   - Her simülasyon adımında ambulans konumunu kontrol et
   - Trafik ışıklarına olan mesafeyi hesapla
   - Yaklaşma mesafesi: 100-200 metre

2. **Işık Kontrol Stratejisi:**
   - Ambulans algılandığında → Yeşil yapmaya hazırla
   - Ambulans yaklaştığında → Yeşil yap
   - Ambulans geçtikten sonra → Normal programa dön

3. **Güvenlik Protokolü:**
   - Ani değişiklikleri önle (minimum sarı ışık süresi)
   - Çakışan yönleri kontrol et
   - Acil durum bitiminde düzgün geçiş

### Veri Yapıları

```python
class EmergencyVehicle:
    - vehicle_id: str
    - position: (x, y)
    - route: List[edge_id]
    - priority: int
    - status: str

class TrafficLight:
    - light_id: str
    - controlled_edges: List[str]
    - current_phase: int
    - normal_program: str
    - emergency_program: str
```

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