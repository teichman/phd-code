all: git
	cd git/Platform/Linux/Build && make -j`cat /proc/cpuinfo  | grep processor | wc -l`

git:
	git clone https://github.com/OpenNI/OpenNI.git git

clean:
	-cd git/Platform/Linux/Build && make clean

wipe:
	-rm -rf git