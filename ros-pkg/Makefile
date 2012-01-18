all:
	rosmake dst

clean:
	for dir in `find . -maxdepth 1 -type d  | egrep -v '^\.$$'`; do echo $$dir; cd $$dir && make clean && cd -; done

wipe:
	for dir in `find . -maxdepth 1 -type d  | egrep -v '^\.$$'`; do echo $$dir; cd $$dir && make wipe && cd -; done

