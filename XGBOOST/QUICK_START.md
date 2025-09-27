# Hızlı Başlangıç Rehberi

Bu rehber, projeyi hızlıca kurup çalıştırmanız için temel adımları içerir.

## 1. Kurulum (5 dakika)

### Gereksinimler
```bash
# Python 3.8+ kurulu olmalı
python --version

# SUMO kurulumunu kontrol et
sumo --version
```

### Bağımlılıkları Yükle
```bash
pip install -r requirements.txt
```

## 2. İlk Çalıştırma (Test Modu)

### Basit Test Senaryosu
```bash
# 1. Test haritası oluştur (basit 4-yollu kavşak)
python scripts/create_test_network.py

# 2. Test simülasyonunu çalıştır
python src/main.py --test-mode

# 3. GUI ile görüntüle
sumo-gui -c config/test_simulation.sumocfg
```

## 3. Burdur-Bucak Haritası (10 dakika)

### Harita İndir
1. [Overpass Turbo](https://overpass-turbo.eu/) sitesine git
2. Burdur-Bucak bölgesini seç
3. Bu sorguyu çalıştır:
```
[out:xml][timeout:25];
(
  way["highway"]($bbox);
);
out geom;
```
4. `maps/burdur-bucak.osm` olarak kaydet

### Haritayı Dönüştür
```bash
# OSM'yi SUMO formatına çevir
netconvert --osm-files maps/burdur-bucak.osm \
           --output-file maps/burdur-bucak.net.xml \
           --tls.guess-signals

# Rotaları oluştur
python scripts/generate_routes.py maps/burdur-bucak.net.xml
```

## 4. Acil Durum Simülasyonu Çalıştır

```bash
# Tam simülasyonu başlat
python src/main.py --config config/burdur_simulation.sumocfg --gui

# Logları izle
tail -f simulation.log
```

## 5. Sonuçları Görüntüle

### Simülasyon sırasında:
- **Kırmızı araçlar**: Ambulanslar
- **Yeşil ışıklar**: Acil durum modu aktif
- **Konsol**: Gerçek zamanlı log mesajları

### Simülasyon sonrası:
```bash
# İstatistikleri görüntüle
python scripts/analyze_results.py data/simulation_output.xml
```

## Hızlı Komutlar

```bash
# Projeyi sıfırla
python scripts/reset_project.py

# Test verilerini oluştur
python scripts/generate_test_data.py

# Simülasyonu durdur
Ctrl+C  # Terminal'de

# Hata loglarını görüntüle
cat logs/error.log
```

## Yaygın Sorunlar

### SUMO bulunamıyor
```bash
# Windows için
set SUMO_HOME=C:\Program Files (x86)\Eclipse\Sumo
set PATH=%PATH%;%SUMO_HOME%\bin

# Linux/Mac için
export SUMO_HOME=/usr/share/sumo
export PATH=$PATH:$SUMO_HOME/bin
```

### Port çakışması
```bash
# Farklı port kullan
python src/main.py --port 8814
```

### Ambulans görünmüyor
Kontrol listesi:
- [ ] `config/routes.xml` içinde ambulans tanımlı mı?
- [ ] Ambulans rotası doğru mu?
- [ ] `vClass="emergency"` ayarlanmış mı?

### Işıklar değişmiyor
Kontrol listesi:
- [ ] Trafik ışığı programları doğru mu?
- [ ] TraCI bağlantısı aktif mi?
- [ ] Mesafe hesaplama doğru mu?

## İleri Özellikler (Opsiyonel)

### Gerçek GPS Verisi
```python
# GPS simülatörünü etkinleştir
python src/gps_simulator.py --real-data
```

### Web Dashboard
```bash
# Web arayüzünü başlat
python web/dashboard.py
# http://localhost:5000
```

### Veri Analizi
```python
# Performans raporunu oluştur
python scripts/generate_report.py
```

## Yardım

- **Dokümantasyon**: `docs/` klasörü
- **Örnekler**: `examples/` klasörü
- **Sorun giderme**: `docs/troubleshooting.md`
- **API referansı**: `docs/api-reference.md`

## Sonraki Adımlar

1. ✅ Temel sistemi çalıştır
2. ⬜ Kendi haritanı ekle
3. ⬜ Çoklu ambulans testi yap
4. ⬜ Performans optimizasyonu
5. ⬜ Gerçek GPS entegrasyonu 