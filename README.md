# bvh

This is a DIY BVH (Bounding Volume Hierarchy) implimentation library (still being optimised),
which means you are adviced to provide a custom primitive type, and the library will make the bvh for you

Suggestion: Look at the examples in this order (to avoid information overload):
    example_basic_usage
    example_bvh_instancing
    example_bvh_tlas
    example_triangle_triangle_intersection

## Build instructions
This library uses cmake to build, 
This library is dependent on glm.
Atm, the cmake file is very basic and will only compile the examples, so dont just include the project in your own cmake based projects as it would not compile.

`cmake -S. -Bbuild`

`cd build`

`make`

`cd ..`

