#!/usr/bin/env bash

if [ $# -ne 1 ]
then
    echo "Usage: $0 <entry_container_name>"
    exit 1
fi

docker run --rm --name $1 subt_entry