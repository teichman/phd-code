all: teichman-ros-pkg
	rosmake core external

teichman-ros-pkg:
	git clone git://github.com/teichman/teichman-ros-pkg.git

clean: 
	rosmake --target=clean core external

wipe: clean
	rosmake --target=wipe core external

