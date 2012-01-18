TARBALL = build/qpOASES-3.0beta.tar.gz
TARBALL_URL = http://www.qpoases.org/download/qpOASES-3.0beta.tar.gz
SOURCE_DIR = build/qpOASES-3.0beta
TARBALL_PATCH = patchfile3

all: $(SOURCE_DIR)/unpacked
	cd build/qpOASES-3.0beta/src && make -f Makefile
	-rm -rf include
	-mkdir include
	cd include && ln -s ../build/qpOASES-3.0beta/include qpOASES

include $(shell rospack find mk)/download_unpack_build.mk

clean: 
	-rm -rf $(SOURCE_DIR) include

wipe: clean
	-rm -rf build
