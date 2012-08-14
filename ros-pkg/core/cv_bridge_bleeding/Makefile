FILES_TO_COPY := `ls $$(rospack find cv_bridge) | grep -v Makefile | grep -v CMakeLists.txt | grep -v manifest.xml | grep -v build | grep -v bin | grep -v lib`
built:
	echo ${FILES_TO_COPY}
	@for f in ${FILES_TO_COPY}; do \
	ln -sf `rospack find cv_bridge`/$${f} .; \
	done
	make -f `rospack find cv_bridge`/Makefile
	@for f in ${FILES_TO_COPY}; do \
	unlink $${f}; \
	done
	touch $@

clean:
	echo ${FILES_TO_COPY}
	@for f in ${FILES_TO_COPY}; do \
	ln -sf `rospack find cv_bridge`/$${f} .; \
	done
	make -f `rospack find cv_bridge`/Makefile clean
	@for f in ${FILES_TO_COPY}; do \
	unlink $${f}; \
	done





