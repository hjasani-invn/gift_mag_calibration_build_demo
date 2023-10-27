#!/bin/sh
for file in  `find $1  -regex '.*\.zip'`
do 
 #echo `dirname ${file}`
 #echo ${file} 
 #unzip -n  ${file} -d `dirname ${file}`  # new files only
 unzip -u -o  ${file} -d `dirname ${file}`  # update files
done
