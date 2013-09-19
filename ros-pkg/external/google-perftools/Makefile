TARBALL = build/gperftools-2.1.tar.gz
TARBALL_URL = http://gperftools.googlecode.com/files/gperftools-2.1.tar.gz
SOURCE_DIR = build/gperftools-2.1

# TARBALL = build/gperftools-2.0.tar.gz 
# TARBALL_URL = http://gperftools.googlecode.com/files/gperftools-2.0.tar.gz
# SOURCE_DIR = build/gperftools-2.0

all: installed

include $(shell rospack find mk)/download_unpack_build.mk

installed: $(SOURCE_DIR)/unpacked
	cd $(SOURCE_DIR) && ./configure --enable-frame-pointers && make -j$(cat /proc/cpuinfo  | grep processor | wc -l)
	mkdir -p bin
	cp $(SOURCE_DIR)/src/pprof bin/ #Can't symlink or rosrun won't accept it.
	touch $@

clean: 
	-rm -rf $(SOURCE_DIR) installed bin/

wipe: clean
	-rm -rf build
