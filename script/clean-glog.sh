#!/bin/bash

for filename in /home/maghob/root/cartographer_android/cartographer/**/**/*.cc; do
    echo "$filename"
    sed '/logging.h/ s?^?//?' $filename > text.txt
    mv text.txt $filename
done

for filename in /home/maghob/root/cartographer_android/cartographer/**/**/*.h; do
    #export file= $filename"_mg"
    echo "$filename"
    sed '/logging.h/ s?^?//?' $filename > text.txt
    mv text.txt $filename
done

for filename in /home/maghob/root/cartographer_android/cartographer/**/*.cc; do
    #export file= $filename"_mg"
    echo "$filename"
    sed '/logging.h/ s?^?//?' $filename > text.txt
    mv text.txt $filename
done

for filename in /home/maghob/root/cartographer_android/cartographer/**/*.h; do
    #export file= $filename"_mg"
    echo "$filename"
    sed '/logging.h/ s?^?//?' $filename > text.txt
    mv text.txt $filename
done
