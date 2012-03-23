#!/bin/bash

stty -echo
read -p "Password: " PASSWD
stty echo
echo

while [ 1 ]; do
    date
    ls .sentinel* &> /dev/null
    if [ $? = "0" ]; then
	echo Uploading data to server.
	scripts/upload.exp $PASSWD .sentinel*
    fi
    SEC=$(( $RANDOM / 250 ))
    echo Sleeping for $SEC seconds
    sleep $SEC
done

