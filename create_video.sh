#!/bin/sh
echo "Enter output file name"
read FILE_NAME
ffmpeg -r 24 -i frame_%10d.png -c:v libx264 -vf "pad=ceil(iw/2)*2:ceil(ih/2)*2" -pix_fmt yuv420p "$FILE_NAME.mp4"