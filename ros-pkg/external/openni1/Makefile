
built: git
	cd git/Platform/Linux/CreateRedist; \
	if [ `arch` = "armv7l" ]; then \
	./RedistMaker Arm; \
	else \
	not_implemented; \
	fi
	cd git-driver/Platform/Linux/CreateRedist; \
	if [ `arch` = "armv7l" ]; then \
	./RedistMaker Arm; \
	else \
	not_implemented; \
	fi
	touch $@

built-old: git
	cd git/Platform/Linux/Build; \
	if [ `arch` = "armv7l" ]; then \
	PLATFORM=Arm make; \
	else \
	make; \
	fi
	touch $@

git:
	git clone https://github.com/teichman/OpenNI.git $@
	cd $@ && git checkout arm

git-driver:
	git clone https://github.com/teichman/Sensor.git $@
	cd $@ && git checkout arm

install: built
	cd git/Platform/Linux/Redist/OpenNI-Bin-Dev-Linux-Arm-v1.5.2.23/ && sudo ./install.sh
	cd git-driver/Platform/Linux/Redist/Sensor-Bin-Linux-Arm-v5.1.0.41/ && sudo ./install.sh

clean:
	rm -f built

wipe: clean
	rm -rf git
