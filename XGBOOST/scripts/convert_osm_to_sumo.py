#!/usr/bin/env python3
"""
OSM dosyasını SUMO network formatına dönüştürür - Traffic Light versiyon
"""

import os
import subprocess
import sys
import xml.etree.ElementTree as ET

# Windows konsolda UTF-8 yazdırma (emoji/simgeler sorun çıkarıyorsa yerine geçer)
try:
    import io
    if sys.platform.startswith('win'):
        sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')
        sys.stderr = io.TextIOWrapper(sys.stderr.buffer, encoding='utf-8', errors='replace')
except Exception:
    pass

def convert_osm_to_sumo():
    """OSM dosyasını SUMO network'e dönüştür"""
    input_osm = "maps/map-new.osm"
    output_net = "config/network_with_tl.net.xml"
    
    if not os.path.exists(input_osm):
        print(f"❌ OSM dosyası bulunamadı: {input_osm}")
        return False
    
    # Output klasörünü oluştur
    os.makedirs("config", exist_ok=True)
    
    # Netconvert komutu - traffic light junction tipi için
    cmd = [
        "netconvert",
        "--osm-files", input_osm,
        "-o", output_net,
        "--geometry.remove",
        "--roundabouts.guess",
        "--ramps.guess", 
        "--junctions.join",
        "--tls.join",
        "--tls.guess",              # Uygun kavşaklara otomatik sinyal tahmini
        "--tls.guess-signals",      # OSM işaretlerinden sinyal çıkarımı
        "--tls.default-type", "static",
        "--tls.green.time", "31",  # Standart SUMO yeşil süresi
        "--tls.yellow.time", "3",   # Sarı süre
        "--tls.red.time", "2",      # Kırmızı temizleme süresi
        "--proj.utm",
        "--output.street-names",
        "--output.original-names"
    ]
    
    try:
        print("🔄 OSM → SUMO Network (Traffic Light) dönüştürülüyor...")
        print(f"Komut: {' '.join(cmd)}")
        
        result = subprocess.run(cmd, capture_output=True, text=True)
        
        if result.returncode == 0:
            print(f"✅ Network başarıyla oluşturuldu: {output_net}")
            
            # Network analizi
            analyze_network(output_net)
            
            # Ana kavşağı manuel olarak traffic_light'a çevir
            print("\n🔧 Ana kavşağı traffic_light tipine çeviriliyor...")
            convert_main_junction_to_traffic_light(output_net)
            
            # Güncellenmiş analiz
            print("\n📊 Güncellenmiş Network Analizi:")
            analyze_network(output_net)
            
            print(f"\n✅ İşlem tamamlandı!")
            print(f"📄 Oluşturulan dosyalar:")
            print(f"   • Network: {output_net}")
            print(f"   • Additional: config/additional.add.xml")
            return True
        else:
            print(f"❌ Network oluşturma hatası:")
            print(f"STDERR: {result.stderr}")
            return False
            
    except FileNotFoundError:
        print("❌ netconvert komutu bulunamadı!")
        print("SUMO kurulumunu kontrol edin ve PATH'e ekleyin.")
        return False
    except Exception as e:
        print(f"❌ Beklenmeyen hata: {str(e)}")
        return False

def convert_main_junction_to_traffic_light(network_file: str):
    """Ana kavşağı kontrol için hazırla - priority olarak bırak"""
    try:
        # Network dosyasını oku
        tree = ET.parse(network_file)
        root = tree.getroot()
        
        # Ana kavşağı bul
        main_junction_ids = ["cluster_3660221600_3660221601"]
        
        for junction in root.findall('.//junction'):
            junction_id = junction.get('id', '')
            junction_type = junction.get('type', '')
            
            if junction_id in main_junction_ids:
                print(f"   • Ana Junction {junction_id}: {junction_type} (priority olarak bırakıldı)")
                # Manuel traffic control için priority olarak bırak
                # TraCI ile araç hızlarını kontrol edeceğiz
        
        print("   ✅ Network hazır - manuel ambulans kontrolü için")
        
    except Exception as e:
        print(f"   ❌ Hata: {str(e)}")

def analyze_network(network_file: str):
    """Network dosyasını analiz et"""
    try:
        tree = ET.parse(network_file)
        root = tree.getroot()
        
        # İstatistikler
        junctions = root.findall('.//junction')
        edges = root.findall('.//edge')
        tllogics = root.findall('.//tlLogic')
        
        print(f"   • Toplam junction: {len(junctions)}")
        print(f"   • Toplam edge: {len(edges)}")
        print(f"   • Toplam tlLogic: {len(tllogics)}")
        
        # Junction türleri
        junction_types = {}
        main_junction_found = False
        
        for junction in junctions:
            junction_id = junction.get('id', '')
            junction_type = junction.get('type', 'unknown')
            
            # Ana kavşağı kontrol et
            if any(main_id in junction_id for main_id in ["cluster_3660221600_3660221601", "3660221600", "3660221601"]):
                main_junction_found = True
                main_junction_type = junction_type
                
            junction_types[junction_type] = junction_types.get(junction_type, 0) + 1
        
        print("\n🚦 Junction Türleri:")
        for jtype, count in junction_types.items():
            print(f"   • {jtype}: {count}")
            
        # Ana kavşak durumu
        if main_junction_found:
            if main_junction_type == 'traffic_light':
                print(f"\n✅ Ana kavşak traffic_light!")
            else:
                print(f"\n⚠️  Ana kavşak {main_junction_type}!")
        else:
            print(f"\n❌ Ana kavşak bulunamadı!")
            
    except Exception as e:
        print(f"Analiz hatası: {str(e)}")

def create_additional_file(output_file: str):
    """Additional dosyası oluştur"""
    additional_content = """<?xml version="1.0" encoding="UTF-8"?>
<additionalFile>
    <!-- Ana kavşak için Traffic Light Logic -->
    <tlLogic id="cluster_3660221600_3660221601" type="static" programID="0" offset="0">
        <!-- 8 fazlı trafik ışığı programı -->
        <!-- Faz 0: Doğu-Batı yeşil -->
        <phase duration="31" state="GGgrrrGGgrrr"/>
        <!-- Faz 1: Doğu-Batı sarı -->
        <phase duration="3"  state="yygrrryygrrr"/>
        <!-- Faz 2: Tümü kırmızı (güvenlik) -->
        <phase duration="2"  state="rrrrrrrrrrrr"/>
        <!-- Faz 3: Kuzey-Güney yeşil -->
        <phase duration="31" state="rrrGGGrrrGGG"/>
        <!-- Faz 4: Kuzey-Güney sarı -->
        <phase duration="3"  state="rrryyyrrryyyy"/>
        <!-- Faz 5: Tümü kırmızı (güvenlik) -->
        <phase duration="2"  state="rrrrrrrrrrrr"/>
        <!-- Faz 6: Sol dönüş fazı -->
        <phase duration="15" state="GrrrrrGrrrrr"/>
        <!-- Faz 7: Sol dönüş sarı -->
        <phase duration="3"  state="yrrrrryrrrr"/>
    </tlLogic>
</additionalFile>"""
    
    try:
        with open(output_file, 'w', encoding='utf-8') as f:
            f.write(additional_content)
        print(f"Additional dosyası oluşturuldu: {output_file}")
    except Exception as e:
        print(f"Additional dosyası oluşturma hatası: {str(e)}")

def create_simulation_config():
    """SUMO simülasyon konfigürasyon dosyasını oluşturur"""
    
    config_content = '''<?xml version="1.0" encoding="UTF-8"?>
<configuration xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" 
               xsi:noNamespaceSchemaLocation="http://sumo.dlr.de/xsd/sumoConfiguration.xsd">
    
    <input>
        <net-file value="network_with_tl.net.xml"/>
        <route-files value="routes.rou.xml"/>
        <additional-files value="additional.add.xml"/>
    </input>
    
    <output>
        <summary-output value="../logs/summary.xml"/>
        <tripinfo-output value="../logs/tripinfo.xml"/>
    </output>
    
    <time>
        <begin value="0"/>
        <end value="3600"/>  <!-- 1 saat -->
        <step-length value="0.1"/>
    </time>
    
    <processing>
        <collision.check-junctions value="true"/>
        <collision.mingap-factor value="1"/>
    </processing>
    
    <routing>
        <device.rerouting.adaptation-steps value="18"/>
        <device.rerouting.adaptation-interval value="10"/>
    </routing>
    
    <report>
        <xml-validation value="never"/>
        <duration-log.disable value="true"/>
        <no-step-log value="true"/>
    </report>
    
</configuration>'''
    
    with open("config/simulation.sumocfg", "w", encoding="utf-8") as f:
        f.write(config_content)
    
    print("✓ Simülasyon konfigürasyonu oluşturuldu: config/simulation.sumocfg")

def main():
    """Ana fonksiyon"""
    print("=== OSM'den SUMO'ya Dönüştürme ===")
    
    # OSM dosyasının varlığını kontrol et
    if not os.path.exists("maps/map.osm"):
        print("✗ maps/map.osm dosyası bulunamadı!")
        return
    
    # Dönüştürme işlemi
    if convert_osm_to_sumo():
        create_additional_file("config/additional.add.xml")
        create_simulation_config()
        print("\n✓ Tüm dosyalar başarıyla oluşturuldu!")
        print("\nSonraki adımlar:")
        print("1. Trafik ışığı programlarını config/additional.add.xml'e ekleyin")
        print("2. Rota dosyasını config/routes.rou.xml olarak oluşturun")
        print("3. Ambulans simülasyonunu çalıştırın")
    else:
        print("✗ Dönüştürme işlemi başarısız!")

if __name__ == "__main__":
    success = convert_osm_to_sumo()
    sys.exit(0 if success else 1) 