# Uygulama Rehberi - Adım Adım

## Aşama 1: Harita Hazırlığı

### 1.1 OpenStreetMap Verilerini İndirme

1. **Overpass Turbo kullanarak:**
   - [Overpass Turbo](https://overpass-turbo.eu/) sitesine gidin
   - Burdur-Bucak bölgesini seçin
   - Aşağıdaki sorguyu kullanın:

```xml
[out:xml][timeout:25];
(
  way["highway"]($bbox);
  way["railway"]($bbox);
  relation["type"="route"]["route"="road"]($bbox);
);
out geom;
```

2. **Verileri kaydedin:**
   - Export → XML olarak kaydedin
   - Dosya adı: `maps/burdur-bucak.osm`

### 1.2 SUMO Ağına Dönüştürme

```bash
# OSM dosyasını SUMO ağına dönüştür
netconvert --osm-files maps/burdur-bucak.osm \
           --output-file maps/burdur-bucak.net.xml \
           --geometry.remove \
           --ramps.guess \
           --junctions.join \
           --tls.guess-signals \
           --tls.discard-simple
```

## Aşama 2: Trafik Işıkları Yapılandırması

### 2.1 Trafik Işığı Programları

Ana kavşaklar için trafik ışığı programları tanımlayın:

```xml
<!-- config/traffic_lights.add.xml -->
<additionalFile>
    <tlLogic id="center_junction" type="static" programID="normal" offset="0">
        <phase duration="31" state="GGrrrrGGrrrr"/>
        <phase duration="6"  state="yyrrrryyrrrr"/>
        <phase duration="31" state="rrGGGGrrGGGG"/>
        <phase duration="6"  state="rryyyyyryyyy"/>
    </tlLogic>
    
    <tlLogic id="center_junction" type="static" programID="emergency" offset="0">
        <phase duration="60" state="GGGGGGGGGGr"/>
        <phase duration="6"  state="yyyyyyyyyyy"/>
    </tlLogic>
</additionalFile>
```

## Aşama 3: Araç Tanımlamaları

### 3.1 Araç Türleri

```xml
<!-- config/vehicles.xml -->
<vTypes>
    <!-- Normal arabalar -->
    <vType id="car" accel="2.6" decel="4.5" sigma="0.5" length="5" 
           maxSpeed="50" color="1,1,0"/>
    
    <!-- Ambulans -->
    <vType id="ambulance" accel="3.0" decel="5.0" sigma="0.2" length="7" 
           maxSpeed="80" color="1,0,0" vClass="emergency"/>
</vTypes>
```

### 3.2 Rotalar

```xml
<!-- config/routes.xml -->
<routes>
    <!-- Normal trafik rotaları -->
    <route id="route_north_south" edges="edge1 edge2 edge3"/>
    <route id="route_east_west" edges="edge4 edge5 edge6"/>
    
    <!-- Ambulans rotaları -->
    <route id="ambulance_route_1" edges="edge7 edge8 edge9"/>
    
    <!-- Araç akışları -->
    <flow id="normal_traffic" type="car" route="route_north_south" 
          begin="0" end="3600" number="100"/>
    
    <vehicle id="ambulance_1" type="ambulance" route="ambulance_route_1" 
             depart="300"/>
</routes>
```

## Aşama 4: Python Kontrol Sistemi

### 4.1 Ana Kontrol Sistemi

```python
# src/main.py - Temel yapı
import traci
import sumolib
from traffic_light_controller import TrafficLightController
from emergency_vehicle_detector import EmergencyVehicleDetector

def main():
    # SUMO'yu başlat
    sumoBinary = "sumo-gui"  # veya "sumo"
    sumoCmd = [sumoBinary, "-c", "config/simulation.sumocfg"]
    traci.start(sumoCmd)
    
    # Kontrol sistemlerini başlat
    tl_controller = TrafficLightController()
    ev_detector = EmergencyVehicleDetector()
    
    # Simülasyon döngüsü
    step = 0
    while traci.simulation.getMinExpectedNumber() > 0:
        traci.simulationStep()
        
        # Acil durum araçlarını kontrol et
        emergency_vehicles = ev_detector.detect_emergency_vehicles()
        
        # Trafik ışıklarını güncelle
        for vehicle in emergency_vehicles:
            tl_controller.process_emergency_vehicle(vehicle)
        
        step += 1
    
    traci.close()

if __name__ == "__main__":
    main()
```

### 4.2 Trafik Işığı Kontrolcüsü

```python
# src/traffic_light_controller.py
import traci
import math

class TrafficLightController:
    def __init__(self):
        self.controlled_lights = {}
        self.emergency_states = {}
        self.detection_range = 150  # metre
        
    def get_nearby_traffic_lights(self, vehicle_position, vehicle_route):
        """Ambulansa yakın trafik ışıklarını bul"""
        nearby_lights = []
        
        for tl_id in traci.trafficlight.getIDList():
            tl_position = traci.junction.getPosition(tl_id)
            distance = self.calculate_distance(vehicle_position, tl_position)
            
            if distance <= self.detection_range:
                nearby_lights.append({
                    'id': tl_id,
                    'distance': distance,
                    'position': tl_position
                })
        
        return sorted(nearby_lights, key=lambda x: x['distance'])
    
    def process_emergency_vehicle(self, vehicle_info):
        """Acil durum aracı için trafik ışıklarını kontrol et"""
        vehicle_id = vehicle_info['id']
        position = vehicle_info['position']
        route = vehicle_info['route']
        
        # Yakındaki trafik ışıklarını bul
        nearby_lights = self.get_nearby_traffic_lights(position, route)
        
        for light_info in nearby_lights[:2]:  # En yakın 2 ışık
            light_id = light_info['id']
            distance = light_info['distance']
            
            if distance <= 50:  # Çok yakın - yeşil yap
                self.set_emergency_green(light_id, vehicle_id)
            elif distance <= 100:  # Yaklaşıyor - hazırla
                self.prepare_emergency_green(light_id, vehicle_id)
    
    def set_emergency_green(self, light_id, vehicle_id):
        """Trafik ışığını acil durum için yeşil yap"""
        current_program = traci.trafficlight.getProgram(light_id)
        
        if current_program != "emergency":
            # Mevcut durumu kaydet
            self.emergency_states[light_id] = {
                'original_program': current_program,
                'vehicle_id': vehicle_id,
                'activation_time': traci.simulation.getTime()
            }
            
            # Acil durum programına geç
            traci.trafficlight.setProgram(light_id, "emergency")
            print(f"Trafik ışığı {light_id} ambulans {vehicle_id} için yeşil yapıldı")
    
    def check_emergency_completion(self):
        """Acil durum bitimini kontrol et ve normal duruma döndür"""
        current_time = traci.simulation.getTime()
        completed_emergencies = []
        
        for light_id, emergency_info in self.emergency_states.items():
            vehicle_id = emergency_info['vehicle_id']
            activation_time = emergency_info['activation_time']
            
            # Araç geçti mi kontrol et
            if self.has_vehicle_passed(light_id, vehicle_id):
                # Normal programa dön
                original_program = emergency_info['original_program']
                traci.trafficlight.setProgram(light_id, original_program)
                completed_emergencies.append(light_id)
                print(f"Trafik ışığı {light_id} normal duruma döndü")
        
        # Tamamlanan acil durumları temizle
        for light_id in completed_emergencies:
            del self.emergency_states[light_id]
    
    def calculate_distance(self, pos1, pos2):
        """İki nokta arası mesafe hesapla"""
        return math.sqrt((pos1[0] - pos2[0])**2 + (pos1[1] - pos2[1])**2)
    
    def has_vehicle_passed(self, light_id, vehicle_id):
        """Ambulansın kavşağı geçip geçmediğini kontrol et"""
        try:
            vehicle_position = traci.vehicle.getPosition(vehicle_id)
            light_position = traci.junction.getPosition(light_id)
            
            # Araç ışıktan uzaklaştı mı?
            distance = self.calculate_distance(vehicle_position, light_position)
            return distance > 100
        except:
            # Araç simülasyondan çıktı
            return True
```

## Aşama 5: Simülasyon Yapılandırması

### 5.1 Ana Yapılandırma Dosyası

```xml
<!-- config/simulation.sumocfg -->
<configuration>
    <input>
        <net-file value="../maps/burdur-bucak.net.xml"/>
        <route-files value="routes.xml"/>
        <additional-files value="traffic_lights.add.xml"/>
    </input>
    
    <time>
        <begin value="0"/>
        <end value="3600"/>
        <step-length value="1"/>
    </time>
    
    <processing>
        <ignore-route-errors value="true"/>
    </processing>
    
    <traci_server>
        <remote-port value="8813"/>
    </traci_server>
</configuration>
```

## Aşama 6: Test ve Çalıştırma

### 6.1 Basit Test

```bash
# 1. Ağı test et
sumo -c config/simulation.sumocfg --begin 0 --end 10

# 2. GUI ile test et
sumo-gui -c config/simulation.sumocfg

# 3. Python kontrolü ile çalıştır
python src/main.py
```

### 6.2 Debug ve Monitoring

```python
# src/debug_utils.py
def log_vehicle_info(vehicle_id):
    position = traci.vehicle.getPosition(vehicle_id)
    speed = traci.vehicle.getSpeed(vehicle_id)
    route = traci.vehicle.getRoute(vehicle_id)
    
    print(f"Araç {vehicle_id}: Konum={position}, Hız={speed}, Rota={route}")

def log_traffic_light_state(light_id):
    state = traci.trafficlight.getRedYellowGreenState(light_id)
    program = traci.trafficlight.getProgram(light_id)
    phase = traci.trafficlight.getPhase(light_id)
    
    print(f"Işık {light_id}: Durum={state}, Program={program}, Faz={phase}")
```

## Sonraki Adımlar

1. **Temel sistemi test edin**
2. **Performansı optimize edin**
3. **Çoklu ambulans desteği ekleyin**
4. **Gerçek GPS verilerini entegre edin**
5. **Web arayüzü geliştirin** 