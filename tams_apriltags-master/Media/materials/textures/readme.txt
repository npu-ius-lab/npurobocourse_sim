http://april.eecs.umich.edu/wiki/index.php/AprilTags

Pre-generated tag families:

Each set includes individual PNGs of each tag, a mosaic PNG containing every tag, 
and a ready-to-print postscript file with one tag per page. 

36h11 (recommended) http://april.eecs.umich.edu/software/tag36h11.tgz
36h10 http://april.eecs.umich.edu/software/tag36h10.tgz
25h9 http://april.eecs.umich.edu/software/tag25h9.tgz
16h5 http://april.eecs.umich.edu/software/tag16h5.tgz

Problem is, the 36h11 8x8 images (1 pixel border, 6x6 inner code area)
are unusable in Gazebo, because of heavy texture filtering. Note that
this occurs even with texture "filtering none" in the Ogre materials 
definition.

Solution:

It works a lot better after the images have been upscaled a bit.
Perhaps upscaling by 8x would be best (64x64 texture), but for now
we have used 10X (80x80 image size).

One way to upscaling are the netpmb tools:

tcsh>
foreach i (*.png)
echo $i
pngtopnm $i | pnmscale 10 | pnmtopng > ../large36h11/$i
end

Note: pngtopn $i | pnmscale 8 | pnmtopng > $i will NOT work,
because the tools are so fast that the output still overwrites
the input file... copying to /tmp/$i in between may work.

TODO:

For now, only the 36h11 tags have been upscaled and integrated into
the xacro/launch files. Feel free to upscale the smaller code tags
also, and add to the xacro and launch files, and the materials-file
generator.

RVIZ refuses to display the Tags that are defined by the Gazebo Tag, so we add a Texture. For some unknown reason, it needs borders around it, for 100x100 Markers, we get a resulting 250x200 image. (for whatever reason)	

#!/bin/bash
mkdir -p ../rviz/36h11
for i in *.png
do
	echo $i
	j=`echo $i | sed 's/tag36_11_0*//g'`
	#pngtopnm $i | pnmscale 10 | pnmtopng > ../large36h11/$i
	pngtopnm $i | pnmscale 10 | pnmtopng | convert : -border 75x50 ../rviz/36h11/$j
done
