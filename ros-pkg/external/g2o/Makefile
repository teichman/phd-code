
build:
	cd svn/trunk && mkdir -p build
	cd svn/trunk/build && cmake ../
	cd svn/trunk/build && make

svn:
	svn co https://svn.openslam.org/data/svn/g2o $@
	cd svn/trunk && patch -p0 ../../patch-2012-04-26

