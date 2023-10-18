^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package geometric_shapes
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.7.3 (2021-05-19)
------------------
* [fix]   Fix memory leak (`#168 <https://github.com/ros-planning/geometric_shapes/issues/168>`_)
* [fix]   Use proper Eigen alignment for make_shared calls (`#187 <https://github.com/ros-planning/geometric_shapes/issues/187>`_)
* [maint] Migrate from Travis to GitHub Actions (`#171 <https://github.com/ros-planning/geometric_shapes/issues/171>`_)
* Contributors: Robert Haschke, Tyler Weaver

0.7.2 (2020-09-25)
------------------
* [maint] Renamed SolidPrimitiveDimCount<shape>::value -> solidPrimitiveDimCount<shape>() (`#121 <https://github.com/ros-planning/geometric_shapes/issues/121>`_)
* [maint] cmake: Consistently use uppercase letters for QHULL dependency
* [maint] cmake: Fix assimp warning
* [maint] Update build badges for Noetic
* Contributors: Robert Haschke

0.7.1 (2020-08-31)
------------------
* [maint] Declare external includes as SYSTEM includes
* [maint] Migration to reentrant qhull (`#149 <https://github.com/ros-planning/geometric_shapes/issues/149>`_)
* [maint] Use soname version for library (`#157 <https://github.com/ros-planning/geometric_shapes/issues/157>`_)
* Contributors: Jochen Sprickerhof, Robert Haschke, Tyler Weaver

0.7.0 (2020-05-25)
------------------
* [feature] Added constructShapeFromBody() and constructMarkerFromBody() (`#138 <https://github.com/ros-planning/geometric_shapes/issues/138>`_)
* [maint]   API cleanup
  * Improve inlining
  * ConvexMesh::MeshData as pimpl
  * Reverted ABI compatibility fixups for Melodic: ed4cf1339cf3765ae9ffa6e6fd111a4e342c5fa2, d582479084a10cac53a7f17e29818b3d8be6161e
* Contributors: Martin Pecka, Robert Haschke

0.6.3 (2020-05-25)
------------------
* [maint]   Provide checkIsometry() helper function (`#144 <https://github.com/ros-planning/geometric_shapes/issues/144>`_)
* [maint]   Remove dynamic casts (`#143 <https://github.com/ros-planning/geometric_shapes/issues/143>`_)
* [feature] Added createEmptyBodyFromShapeType() (`#137 <https://github.com/ros-planning/geometric_shapes/issues/137>`_)
  This allows more efficient body construction when scale, padding or pose should also be set during the construction.
* Contributors: Martin Pecka, Michael Görner

0.6.2 (2020-05-02)
------------------
* [maint]   clang-tidy fixes in headers (`#139 <https://github.com/ros-planning/geometric_shapes/issues/139>`_)
* [fix]     Various fixes + performance improvements (`#109 <https://github.com/ros-planning/geometric_shapes/issues/109>`_, `#126 <https://github.com/ros-planning/geometric_shapes/issues/126>`_, `#107 <https://github.com/ros-planning/geometric_shapes/issues/107>`_, `#108 <https://github.com/ros-planning/geometric_shapes/issues/108>`_)
  * Use Eigen::Isometry3d::linear() instead of rotation()
  * Normalize the direction vector passed to Body::intersectsRay() (`#115 <https://github.com/ros-planning/geometric_shapes/issues/115>`_)
  * Improved test coverage
* [feature] Added support for non-uniform scaling and padding of shapes. (`#103 <https://github.com/ros-planning/geometric_shapes/issues/103>`_)
* [maint]   Made bodies::samplePointInside() const. (`#133 <https://github.com/ros-planning/geometric_shapes/issues/133>`_)
* [fix]     Throw runtime exception when a shape or body should have a negative dimension. (`#106 <https://github.com/ros-planning/geometric_shapes/issues/106>`_)
* [maint]   Prefer std::make_shared (`#116 <https://github.com/ros-planning/geometric_shapes/issues/116>`_)
* [maint]   clang-tidy fixes (`#114 <https://github.com/ros-planning/geometric_shapes/issues/114>`_)
* [fix]     Use covariant returns for clone() (`#102 <https://github.com/ros-planning/geometric_shapes/issues/102>`_)
* [feature] Added bodies::Body::computeBoundingBox (aligned box version). (`#104 <https://github.com/ros-planning/geometric_shapes/issues/104>`_)
* [maint]   Windows compatibility: fix ASSIMP libraries path (`#101 <https://github.com/ros-planning/geometric_shapes/issues/101>`_)
* [fix]     Body::containsPoint(): always include surface points (`#97 <https://github.com/ros-planning/geometric_shapes/issues/97>`_)
* Contributors: Martin Pecka, Alejandro Hernández Cordero, Bryce Willey, Michael Görner, Mike Lautman, Robert Haschke, RoboticsYY, Sean Yen, Tyler Weaver

0.6.1 (2018-12-09)
------------------
* Limit minimum number of cylinder vertices (on circumference) to 6 (`#92 <https://github.com/ros-planning/geometric_shapes/issues/92>`_)
* Eigen::Affine3d -> Eigen::Isometry3d (`#88 <https://github.com/ros-planning/geometric_shapes/issues/88>`_)
* Contributors: Robert Haschke, eisoku9618

0.6.0 (2018-05-14)
------------------
* Add method getPlanes and use double precision for planes (`#82 <https://github.com/ros-planning/geometric_shapes/issues/82>`_)
* Contributors: Bence Magyar

0.5.4 (2018-04-06)
------------------
* gracefully handle negative cylinder height: `#64 <https://github.com/ros-planning/geometric_shapes/issues/64>`_, `#80 <https://github.com/ros-planning/geometric_shapes/issues/80>`_
* clang-formatting of whole repo: `#79 <https://github.com/ros-planning/geometric_shapes/issues/79>`_
* operator<< for ShapeType: `#80 <https://github.com/ros-planning/geometric_shapes/issues/80>`_
* adaption to new CONSOLE_BRIDGE_logXXX API: `#75 <https://github.com/ros-planning/geometric_shapes/issues/75>`_, `#72 <https://github.com/ros-planning/geometric_shapes/issues/72>`_
* [fix] box-ray intersection: `#73 <https://github.com/ros-planning/geometric_shapes/issues/73>`_
* Contributors: Dave Coleman, Leroy Rügemer, Malcolm Mielle, Mike Purvis, Robert Haschke, Michael Goerner

0.5.3 (2017-11-26)
------------------
* [enhance] Add warning about common Assimp bug (`#63 <https://github.com/ros-planning/geometric_shapes/issues/63>`_)
* [maintenance] Update maintainers (`#66 <https://github.com/ros-planning/geometric_shapes/issues/66>`_)
* Contributors: Dave Coleman

0.5.2 (2016-10-20)
------------------
* [fix] mesh with too many vertices (`#39 <https://github.com/ros-planning/geometric_shapes/issues/39>`_) (`#60 <https://github.com/ros-planning/geometric_shapes/issues/60>`_)
* [fix] gcc6 build error (`#56 <https://github.com/ros-planning/geometric_shapes/issues/56>`_)
* [fix] Clear root transformation on imported Collada meshes. `#52 <https://github.com/ros-planning/geometric_shapes/issues/52>`_
* [improve] relax mesh containment test (`#58 <https://github.com/ros-planning/geometric_shapes/issues/58>`_)
* [maintenance] Switch boost::shared_ptr to std::shared_ptr. `#57 <https://github.com/ros-planning/geometric_shapes/pull/57>`_
* Contributors: Dave Coleman, Isaac I.Y. Saito, Lukas Bulwahn, Maarten de Vries, Michael Goerner

0.5.1 (2016-08-23)
------------------
* add c++11 safe-guards to the respective headers (`#51 <https://github.com/ros-planning/geometric_shapes/issues/51>`_)
  This is, to be polite and point problems that might arise it out to users.
* Fix incorrect hint always sent to Assimp, improved STL reading (`#24 <https://github.com/ros-planning/geometric_shapes/issues/24>`_)
* Contributors: Dave Coleman, Michael Görner

0.5.0 (2016-07-31)
------------------
* [fix] append cmake module path instead of prepending (`#22 <https://github.com/ros-planning/geometric_shapes/issues/22>`_)
* [fix] FindQhull with non-debian systems (`#30 <https://github.com/ros-planning/geometric_shapes/issues/30>`_). See https://github.com/PointCloudLibrary/pcl/pull/852
* [sys] Use std::shared_ptr for compatibility with FCL 0.5. `#47 <https://github.com/ros-planning/geometric_shapes/issues/47>`_
* [sys] Switch to eigen 3 (`#46 <https://github.com/ros-planning/geometric_shapes/issues/46>`_)
* [sys] Switched to C++11 `#44 <https://github.com/ros-planning/geometric_shapes/issues/44>`_
* [sys] add notice that project will be built in Release mode
* [sys] Remove link_directories, deprecated assimp code
* Contributors: Dave Coleman, Ioan A Sucan, Jochen Sprickerhof, Maarten de Vries, Michael Goerner

0.4.4 (2016-03-06)
------------------
* Merge pull request `#37 <https://github.com/ros-planning/geometric_shapes/issues/37>`_ from corot/indigo-devel
  Fix issue `#28 <https://github.com/ros-planning/geometric_shapes/issues/28>`_ on small radius cylinders
* Contributors: Dave Coleman, Jorge Santos Simon

0.4.3 (2015-04-30)
------------------
* add functions for better display of convex meshes
* produce actual triangles for qhull mesh
* Fixed inverted scale for convex meshes inside check
* Contributors: Christian Dornhege, Michael Ferguson

0.4.2 (2015-04-22)
------------------
* PR `#32 <https://github.com/ros-planning/geometric_shapes/issues/32>`_
  Merge shape_tools package into geometric shapes
* PR `#33 <https://github.com/ros-planning/geometric_shapes/issues/33>`_
  Add run_depend on visualization_msgs
* PR `#26 <https://github.com/ros-planning/geometric_shapes/issues/26>`_
  Prevent every mesh generation opening a new file handle.
* Contributors: Christian Dornhege, Dave Coleman, Jochen Sprickerhof, Michael Ferguson, Steven Peters

0.4.1 (2014-07-07)
------------------
* update distro for travis testing. precise:=trusty
* update to use debian console_bridge dependency. https://github.com/ros/rosdistro/issues/4633
* Contributors: Ioan A Sucan, Tully Foote

0.4.0 (2014-06-24)
------------------
* update usage of console_bridge to deal with version in Trusty
* Merge pull request `#13 <https://github.com/ros-planning/geometric_shapes/issues/13>`_ from ros-planning/testing-in-travis
  Run local and moveit_core tests in Travis builds.
* Merge pull request `#18 <https://github.com/ros-planning/geometric_shapes/issues/18>`_ from dirk-thomas/hydro-devel
  fix configure config.h.in when paths contain spaces fix `#9 <https://github.com/ros-planning/geometric_shapes/issues/9>`_
* Run local and moveit_core tests in Travis builds.
* Contributors: Acorn, Dave Hershberger, Dirk Thomas, Ioan A Sucan, William Woodall

0.3.8 (2014-02-25)
------------------
* fix how we find eigen
* Contributors: Ioan Sucan

0.3.7 (2014-02-23)
------------------
* add build dep so we can find eigen, build fixes
* Contributors: Ioan A Sucan, Scott K Logan

0.3.6 (2014-01-31)
------------------
* Use assimp-dev dep for building
* Remove stray IS_ASSIMP3 define
* Invert Assimp version detect logic for greater accuracy
* Better feature detection for assimp version
* added travis support
* check for CATKIN_ENABLE_TESTING
* Contributors: Dave Hershberger, Ioan A Sucan, Lukas Bulwahn, Scott K Logan

0.3.5 (2013-09-23)
------------------
* Fix syntax error.
* white space fixes (tabs are now spaces)
* add comments for shape definitions
