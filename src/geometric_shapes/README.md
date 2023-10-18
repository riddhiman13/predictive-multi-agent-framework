# Geometric Shapes

This package contains generic definitions of geometric shapes and bodies, as well as tools for operating on shape messages.
Shapes represent only the form of an object.
Bodies are shapes at a particular pose. Routines such as point containment and ray intersections are provided.

Supported shapes:
- sphere
- box
- cone
- cylinder
- mesh

Note: Bodies for meshes compute the convex hull of those meshes in order to provide the point containment / ray intersection routines.

Note: [shape_tools](https://github.com/ros-planning/shape_tools) package was recently merged into this package

Note: `bodies::Box::corner1_` was renamed to `minCorner_` and `bodies::Box::corner2_` to `maxCorner_` in Noetic.

Note: `bodies::ConvexMesh::MeshData` was made implementation-private in Noetic and is no longer accessible from the `.h` file.

## Build Status

GitHub Actions:
[![Format](https://github.com/ros-planning/geometric_shapes/actions/workflows/format.yaml/badge.svg?branch=noetic-devel)](https://github.com/ros-planning/geometric_shapes/actions/workflows/format.yaml?query=branch%3Anoetic-devel)
[![CI](https://github.com/ros-planning/geometric_shapes/actions/workflows/ci.yaml/badge.svg?branch=noetic-devel)](https://github.com/ros-planning/geometric_shapes/actions/workflows/ci.yaml?query=branch%3Anoetic-devel)

Devel Job: [![Build Status](http://build.ros.org/buildStatus/icon?job=Nsrc_uF__geometric_shapes__ubuntu_focal__source)](http://build.ros.org/view/Nsrc_uF/job/Nsrc_uF__geometric_shapes__ubuntu_focal__source)

Debian Job: [![Build Status](http://build.ros.org/buildStatus/icon?job=Nbin_uF64__geometric_shapes__ubuntu_focal_amd64__binary)](http://build.ros.org/view/Nbin_uF64/job/Nbin_uF64__geometric_shapes__ubuntu_focal_amd64__binary)
