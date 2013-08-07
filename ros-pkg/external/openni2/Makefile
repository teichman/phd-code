
built: git
	cd git; \
	if [ `arch` = "armv7l" ]; then \
	PLATFORM=Arm make -j`cat /proc/cpuinfo  | grep processor | wc -l`; \
	else \
	make -j`cat /proc/cpuinfo  | grep processor | wc -l`; \
	fi
	touch $@

git:
	@git clone https://github.com/teichman/OpenNI2.git $@; \
	RES=$$?; \
	sleep 1; \
	if [ $$RES -ne 0 ]; then \
	echo Using ssh fallback.; \
	git clone git@github.com:teichman/OpenNI2.git $@; \
	fi

clean:
	rm -f built

wipe: clean
	rm -rf git

install: built
	sudo ./git/Packaging/Linux/install.sh