#!/bin/bash

for filename in /home/maghob/root/cartographer_android/cartographer/**/**/*.cc; do
    #export file= $filename"_mg"
    echo "$filename"
    sed '/CHECK/ s?^?//?' $filename > text.cc
    mv text.cc $filename
done

#for file in ../cartographer/**/*  ; do echo "$file is a directory"; done
