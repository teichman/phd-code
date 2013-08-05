
built: git
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
