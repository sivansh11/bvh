# bvh

This is a generic BVH (Bounding Volume Hierarchy) implimentation library (still being optimised),
which means you can provide a custom primitive type (could just be a triangle with extra intersection data), and the library will make the bvh for you

Look at src/main.cpp for example

## Build instructions
This library uses cmake to build, 
This library is dependent on glm.

`cmake -S. -Bbuild`

`cd build`

`make`

`cd ..`

## To run
in project root folder
`build/app > image.ppm`
