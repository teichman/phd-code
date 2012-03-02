all:
	@echo Making packages:
	@echo
	@echo `find * -maxdepth 0 -type d` | xargs -n1
	@echo
	rosmake -s `find * -maxdepth 0 -type d`

clean:
	@for dir in `find * -maxdepth 0 -type d`; \
	do echo Cleaning $$dir; \
	cd $$dir; \
	make clean 2>/dev/null 1>&2; \
	cd ../; \
	done

wipe:
	@for dir in `find * -maxdepth 0 -type d`; \
	do echo Wiping $$dir; \
	cd $$dir; \
	make wipe 2>/dev/null 1>&2; \
	cd ../; \
	done

