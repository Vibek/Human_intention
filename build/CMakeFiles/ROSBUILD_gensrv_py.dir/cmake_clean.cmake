FILE(REMOVE_RECURSE
  "../msg_gen"
  "../srv_gen"
  "../msg_gen"
  "../srv_gen"
  "../src/skeleton_markers/msg"
  "../src/skeleton_markers/srv"
  "CMakeFiles/ROSBUILD_gensrv_py"
  "../src/skeleton_markers/srv/__init__.py"
  "../src/skeleton_markers/srv/_FollowWall.py"
  "../src/skeleton_markers/srv/_useMazeFollower.py"
  "../src/skeleton_markers/srv/_pathSucceeded.py"
  "../src/skeleton_markers/srv/_checkObjectInMap.py"
  "../src/skeleton_markers/srv/_usePathFollower.py"
  "../src/skeleton_markers/srv/_objects.py"
  "../src/skeleton_markers/srv/_MakeTurn.py"
  "../src/skeleton_markers/srv/_ResetPWM.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
