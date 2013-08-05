
built: git
	cd git/Platform/Linux/CreateRedist; \
	if [ `arch` = "armv7l" ]; then \
	./RedistMaker Arm; \
	cd ../Redist; \
	./install.sh -c ${ARM_STAGING}; \
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

clean:
	rm -f built

wipe: clean
	rm -rf git
