all: install-driver

built-git: git
	cd git/Platform/Linux/CreateRedist; \
	if [ `arch` = "armv7l" ]; then \
	./RedistMaker Arm; \
	else \
	./RedistMaker; \
	fi
	touch $@

built-git-driver: git-driver
	cd git-driver/Platform/Linux/CreateRedist; \
	if [ `arch` = "armv7l" ]; then \
	./RedistMaker Arm; \
	else \
	./RedistMaker; \
	fi
	touch $@

git:
	git clone https://github.com/teichman/OpenNI.git $@
	cd $@ && git checkout arm

git-driver:
	git clone https://github.com/teichman/Sensor.git $@
	cd $@ && git checkout arm

install-driver: built-git-driver install-openni
	cd git-driver/Platform/Linux/Redist/Sensor-Bin-Linux-*/ && sudo ./install.sh
	touch $@

install-openni: built-git
	cd git/Platform/Linux/Redist/OpenNI-Bin-Dev-Linux-*/ && sudo ./install.sh
	touch $@

clean:
	rm -f built-git built-git-driver

wipe: clean
	rm -rf git git-driver
