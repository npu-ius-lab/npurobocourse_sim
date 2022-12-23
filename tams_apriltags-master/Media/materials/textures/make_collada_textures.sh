#!/bin/bash
# upscale original 8x8 markers to 256x256 textures for
# better rendering in meshlab, ogre, rviz, etc.
#
mkdir /tmp/tag16h5
cd tag16h5
for i in *.png
do
  echo $i
  pngtopnm $i | pnmscale 32 | pnmtopng > /tmp/tag16h5/$i
done
cd ..

mkdir /tmp/tag36h11
cd tag36h11
for i in *.png
do
  echo $i
  pngtopnm $i | pnmscale -xysize 512 512 | pnmtopng > /tmp/tag36h11/$i
done
cd ..
   

