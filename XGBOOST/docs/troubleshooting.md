# Sorun Giderme Rehberi

## Yaygın Kurulum Sorunları

### 1. SUMO Kurulum Sorunları

#### Problem: "sumo: command not found"
**Çözüm:**
```bash
# Windows için:
# 1. SUMO_HOME çevre değişkenini kontrol edin
echo $SUMO_HOME

# 2. PATH'e SUMO bin klasörünü ekleyin
export PATH=$PATH:$SUMO_HOME/bin

# 3. Sistemi yeniden başlatın
```

#### Problem: TraCI bağlantı hatası
**Çözüm:**
```python
# Port çakışması durumunda farklı port kullanın
traci.start(sumoCmd, port=8814)  # varsayılan: 8813

# Firewall sorunu için
# Windows Defender'da SUMO'yu beyaz listeye ekleyin
```

### 2. Harita ve Ağ Sorunları

#### Problem: OSM dosyası yüklenmiyor
**Çözüm:**
```bash
# Dosya boyutunu kontrol edin (>100MB ise parçalayın)
# Dosya kodlamasını kontrol edin (UTF-8 olmalı)

# OSM dosyasını doğrulayın
osmconvert burdur-bucak.osm --statistics

# Hatalı veriler varsa temizleyin
osmconvert burdur-bucak.osm --drop-broken-refs -o=clean.osm
```

#### Problem: Trafik ışıkları oluşturulmuyor
**Çözüm:**
```bash
# netconvert parametrelerini kontrol edin
netconvert --osm-files maps/burdur-bucak.osm \
           --output-file maps/burdur-bucak.net.xml \
           --tls.guess-signals true \
           --tls.join-dist 15 \
           --junctions.join true
```

### 3. Simülasyon Sorunları

#### Problem: Ambulans algılanmıyor
**Debug adımları:**
```python
# src/debug_emergency.py
import traci

def debug_emergency_detection():
    # Tüm araçları listele
    all_vehicles = traci.vehicle.getIDList()
    print(f"Tüm araçlar: {all_vehicles}")
    
    # Ambulansları filtrele
    ambulances = [v for v in all_vehicles if 'ambulance' in v.lower()]
    print(f"Ambulanslar: {ambulances}")
    
    # Her ambulans için detay
    for amb in ambulances:
        position = traci.vehicle.getPosition(amb)
        vtype = traci.vehicle.getTypeID(amb)
        print(f"Ambulans {amb}: Konum={position}, Tip={vtype}")
```

#### Problem: Trafik ışıkları değişmiyor
**Çözümler:**

1. **Program ID kontrolü:**
```python
# Mevcut programı kontrol et
current_program = traci.trafficlight.getProgram(light_id)
print(f"Mevcut program: {current_program}")

# Mevcut programları listele
programs = traci.trafficlight.getAllProgramLogics(light_id)
print(f"Kullanılabilir programlar: {[p.programID for p in programs]}")
```

2. **Fazı zorla değiştirme:**
```python
# Acil durum için fazı zorla değiştir
traci.trafficlight.setPhase(light_id, 0)  # Yeşil faz
traci.trafficlight.setPhaseDuration(light_id, 60)  # 60 saniye
```

## Performans Sorunları

### 1. Yavaş Simülasyon

#### Problem: Simülasyon çok yavaş çalışıyor
**Çözümler:**

```python
# Simülasyon hızını artır
traci.simulation.setScale(2.0)  # 2x hız

# GUI'yi kapatarak hızlandır
sumoBinary = "sumo"  # "sumo-gui" yerine

# Adım boyutunu artır (daha az hassasiyet)
# simulation.sumocfg içinde:
# <step-length value="0.1"/>  # varsayılan: 1.0
```

### 2. Bellek Kullanımı

#### Problem: Yüksek bellek kullanımı
**Çözümler:**

```python
# Gereksiz verileri temizle
def cleanup_old_data():
    current_time = traci.simulation.getTime()
    
    # Eski acil durum kayıtlarını temizle
    for light_id in list(emergency_states.keys()):
        if current_time - emergency_states[light_id]['time'] > 300:
            del emergency_states[light_id]

# Her 100 adımda bir temizle
if step % 100 == 0:
    cleanup_old_data()
```

## Algoritma Sorunları

### 1. Yanlış Mesafe Hesaplama

#### Problem: Ambulans-ışık mesafesi yanlış
**Çözüm:**

```python
def improved_distance_calculation(vehicle_pos, junction_pos):
    """Euclidean mesafe yerine rota üzerinden mesafe"""
    try:
        # Araç rotasını al
        route = traci.vehicle.getRoute(vehicle_id)
        route_pos = traci.vehicle.getRouteIndex(vehicle_id)
        
        # Rota üzerinden mesafe hesapla
        remaining_distance = 0
        for i in range(route_pos, len(route)):
            edge_length = traci.lane.getLength(route[i] + "_0")
            remaining_distance += edge_length
            
            # Kavşağa ulaştık mı?
            edge_junction = traci.edge.getToJunction(route[i])
            if edge_junction == junction_id:
                break
                
        return remaining_distance
    except:
        # Hata durumunda Euclidean mesafe kullan
        return math.sqrt((vehicle_pos[0] - junction_pos[0])**2 + 
                        (vehicle_pos[1] - junction_pos[1])**2)
```

### 2. Çakışan Acil Durumlar

#### Problem: Çoklu ambulans çakışması
**Çözüm:**

```python
class EmergencyPriorityManager:
    def __init__(self):
        self.active_emergencies = {}
        
    def handle_multiple_emergencies(self, light_id, vehicles):
        """Çoklu acil durum aracını yönet"""
        if not vehicles:
            return
            
        # Öncelik sıralaması: mesafe, araç tipi
        sorted_vehicles = sorted(vehicles, 
                               key=lambda v: (v['distance'], v['priority']))
        
        # En yüksek öncelikli aracı seç
        primary_vehicle = sorted_vehicles[0]
        
        # Işığı kontrol et
        self.set_emergency_green(light_id, primary_vehicle['id'])
        
        # Diğer araçları beklet
        for vehicle in sorted_vehicles[1:]:
            self.queue_emergency_vehicle(light_id, vehicle)
```

## Veri Doğrulama

### Test Senaryoları

```python
# tests/test_emergency_system.py
import unittest

class TestEmergencySystem(unittest.TestCase):
    
    def test_ambulance_detection(self):
        """Ambulans algılama testi"""
        # Ambulans oluştur
        # Algılama sistemini test et
        pass
    
    def test_traffic_light_change(self):
        """Trafik ışığı değişim testi"""
        # Trafik ışığını test et
        pass
    
    def test_multiple_ambulances(self):
        """Çoklu ambulans testi"""
        # Çoklu ambulans senaryosunu test et
        pass
```

## Logging ve Monitoring

### Detaylı Log Sistemi

```python
import logging

# Logging yapılandırması
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s',
    handlers=[
        logging.FileHandler('simulation.log'),
        logging.StreamHandler()
    ]
)

def log_emergency_event(event_type, vehicle_id, light_id, details):
    """Acil durum olaylarını logla"""
    logging.info(f"EMERGENCY: {event_type} - Vehicle: {vehicle_id}, Light: {light_id}, Details: {details}")

# Kullanım
log_emergency_event("DETECTED", "ambulance_1", "junction_123", "Distance: 150m")
log_emergency_event("LIGHT_CHANGED", "ambulance_1", "junction_123", "Green activated")
```

## İletişim ve Destek

### Sorun Bildirim Formatı

Sorun bildirirken aşağıdaki bilgileri ekleyin:

1. **Sistem Bilgileri:**
   - İşletim sistemi
   - SUMO sürümü
   - Python sürümü

2. **Hata Mesajı:**
   - Tam hata metni
   - Hata öncesi log kayıtları

3. **Tekrar Üretme Adımları:**
   - Hangi dosyalar kullanıldı
   - Hangi komutlar çalıştırıldı
   - Beklenen vs. gerçek sonuç

4. **Ek Dosyalar:**
   - Yapılandırma dosyaları
   - Log dosyaları
   - Ekran görüntüleri 