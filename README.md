# bvh

This is a generic BVH library (still being optimised)

In this library, you can provide a custom primitive type (could just be a triangle with extra intersection data)

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
