#!/usr/bin/env bash

## Params
# $1 = config filepath
# $2 = rosbridge URL
# $3 = rosbridge port

printf "{\n\t\"url\": \"%s\",\n\t\"port\": \"%s\"\n}" $2 $3 >> $1
