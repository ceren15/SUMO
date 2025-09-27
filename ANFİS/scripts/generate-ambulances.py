#!/usr/bin/env python3
"""
Ambulans rotaları üretir (randomTrips) ve config/amb.rou.xml olarak kaydeder.
"""

import os
import subprocess
import sys


def find_random_trips_py() -> str:
	paths = []
	if 'SUMO_HOME' in os.environ:
		paths.append(os.path.join(os.environ['SUMO_HOME'], 'tools', 'randomTrips.py'))
	paths.append(r"C:\Program Files (x86)\Eclipse\Sumo\tools\randomTrips.py")
	paths.append(r"C:\Program Files\Eclipse\Sumo\tools\randomTrips.py")
	for p in paths:
		if os.path.exists(p):
			return p
	return ''


def main():
	net = 'config/network_with_tl.net.xml'
	out = 'config/amb.rou.xml'
	if not os.path.exists(net):
		print(f"❌ Ağ dosyası yok: {net}")
		return 1

	random_trips = find_random_trips_py()
	if not random_trips:
		print('❌ randomTrips.py bulunamadı. SUMO kurulumu ve SUMO_HOME değişkenini kontrol edin.')
		return 1

	os.makedirs('config', exist_ok=True)

	cmd = [
		sys.executable,
		random_trips,
		'-n', net,
		'-r', out,
		'-b', '10',
		'-e', '600',
		'-p', '90',  # her 90 sn'de bir ambulans
		'--seed', '7',
		'--prefix', 'amb',
		'--vtype', 'ambulance',
	]
	print('🔄 Ambulans rotaları üretiliyor...')
	print('CMD:', ' '.join(cmd))
	res = subprocess.run(cmd, capture_output=True, text=True)
	if res.returncode != 0:
		print('❌ randomTrips hatası:')
		print(res.stderr)
		return res.returncode
	print('✅ Ambulans rotaları üretildi:', out)
	return 0


if __name__ == '__main__':
	sys.exit(main())


