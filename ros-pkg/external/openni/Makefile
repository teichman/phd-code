all: git/sensor git/openni
	cd git/openni/Platform/Linux/Build && make -j`cat /proc/cpuinfo  | grep processor | wc -l`
	cd git/sensor/Platform/Linux/Build && make -j`cat /proc/cpuinfo  | grep processor | wc -l`

git/sensor:
	mkdir -p $(@D)
	git clone https://github.com/PrimeSense/Sensor.git git/sensor
	cd $@ && git reset --hard fc51d0a18a3a22b2f45d84e2b33cf20d366a1c37

git/openni:
	mkdir -p $(@D)
	git clone https://github.com/OpenNI/OpenNI.git git/openni
	cd $@ && git reset --hard 3d355acc8e31ec16253336ae9acea769ec088e04

clean:
	-cd git/Platform/Linux/Build && make clean

wipe:
	-rm -rf git