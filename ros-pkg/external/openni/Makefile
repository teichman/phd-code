built: git/sensor git/openni
	cd git/openni/Platform/Linux/CreateRedist && ./RedistMaker
	cd git/sensor/Platform/Linux/CreateRedist && ./RedistMaker
	touch $@
#	cd git/openni/Platform/Linux/Build && make -j`cat /proc/cpuinfo  | grep processor | wc -l`
#	cd git/sensor/Platform/Linux/Build && make -j`cat /proc/cpuinfo  | grep processor | wc -l`


# You'll need to make install in order to actually use the sensor,
# but we don't do this by default because that would be rude.
install: built
	cd git/openni/Platform/Linux/Redist/OpenNI-Bin-Dev-Linux-x64-v1.5.2.23/ && sudo ./install.sh
	cd git/sensor/Platform/Linux/Redist/Sensor-Bin-Linux-x64-v5.1.0.41/ && sudo ./install.sh

uninstall:
	cd git/openni/Platform/Linux/CreateRedist && sudo ./install.sh -u
	cd git/sensor/Platform/Linux/CreateRedist && sudo ./install.sh -u

git/sensor:
	mkdir -p $(@D)
	git clone https://github.com/PrimeSense/Sensor.git git/sensor
	cd $@ && git reset --hard fc51d0a18a3a22b2f45d84e2b33cf20d366a1c37
	cd $@ && patch -p0 < ../../sensor_patch

git/openni:
	mkdir -p $(@D)
	git clone https://github.com/OpenNI/OpenNI.git git/openni
	cd $@ && git reset --hard 3d355acc8e31ec16253336ae9acea769ec088e04

clean:
	-cd git/Platform/Linux/Build && make clean
	-rm built

wipe:
	-rm -rf git built
