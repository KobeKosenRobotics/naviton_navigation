#!/bin/bash          

echo "Now waiting $1 [sec]"
sleep $1
shift 
"$@"