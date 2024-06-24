#!/bin/bash 
/usr/bin/python ./ina_reader.py | awk '/Current/ { 
            if ($3 <= 0) { printf("%s Offline ",$1);offline++ } 
	    else { 
                  printf("%s Online",$1);online++ } 
	    }'
