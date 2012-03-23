#!/bin/bash

stty -echo
read -p "Password: " PASSWD
stty echo
echo

while [ 1 ]; do
    ls .sentinel* &> /dev/null
    date
    if [ $? = "0" ]; then
	echo Uploading data to server.
	scripts/upload.exp $PASSWD .sentinel*
    fi
    SEC=$(( $RANDOM / 250 ))
    echo Sleeping for $SEC
    sleep $SEC
done

