FILES_TO_COPY := `ls $$(rospack find cloud_basics) | grep -v Makefile | grep -v CMakeLists.txt | grep -v manifest.xml | grep -v build | grep -v bin | grep -v lib`
built:
	echo ${FILES_TO_COPY}
	@for f in ${FILES_TO_COPY}; do \
	ln -sf `rospack find cloud_basics`/$${f} .; \
	done
	make -f `rospack find cloud_basics`/Makefile
	@for f in ${FILES_TO_COPY}; do \
	unlink $${f}; \
	done
	touch $@

#built:
#	@for f in conf.py include index.rst src test; do \
#	ln -sf `rospack find cloud_basics`/$${f} .; \
#	done
#	make -f `rospack find cloud_basics`/Makefile
#	@for f in conf.py include index.rst src test; do \
#	unlink $${f}; \
#	done
#	touch $@



