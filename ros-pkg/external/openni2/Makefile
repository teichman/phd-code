
built: git
	cd git; \
	if [ `arch` = "armv7l" ]; then \
	PLATFORM=Arm make; \
	else \
	make; \
	fi
	touch $@

git:
	git clone https://github.com/teichman/OpenNI2.git $@

clean:
	rm -f built

wipe: clean
	rm -rf git
