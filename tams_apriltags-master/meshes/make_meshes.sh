#!/bin/bash
#
# 16h5 family
for i in tag16_05*.png
do
  echo $i
  j=$(basename $i .png).dae
  echo $j
  cp master.dae $j
  sed -i s/master.png/$i/ $j
done

# 36h11 family
for i in tag36_11*.png
do
  echo $i
  j=$(basename $i .png).dae
  echo $j
  cp master.dae $j
  sed -i s/master.png/$i/ $j
done

# other families not yet (not needed)
