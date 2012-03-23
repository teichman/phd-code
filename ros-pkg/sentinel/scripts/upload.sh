#!/bin/bash

stty -echo
read -p "Password: " PASSWD
stty echo
echo

PREV_NUM=0
while [ 1 ]; do
    date
    ls .sentinel* &> /dev/null
    if [ $? != "0" ]; then
	echo No .sentinel file.  Waiting.
	sleep 60
	continue
    fi
    
    NUM=$(find .sentinel* -name '*.png' | wc -l)
    echo Previously: $PREV_NUM.  Now: $NUM.
    if [ $NUM != $PREV_NUM ]; then
	echo Uploading new data to server.
	rosrun sentinel upload.exp $PASSWD .sentinel*
    fi
    PREV_NUM=$NUM
    
    SEC=$(( $RANDOM / 60 ))
    echo Sleeping for $SEC seconds
    sleep $SEC
done

