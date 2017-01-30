FILE(REMOVE_RECURSE
  "../msg_gen"
  "../msg_gen"
  "../src/Human_intention/msg"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "../src/Human_intention/msg/__init__.py"
  "../src/Human_intention/msg/_Skeleton.py"
  "../src/Human_intention/msg/_PerceptInfo.py"
  "../src/Human_intention/msg/_Goal.py"
  "../src/Human_intention/msg/_EnableJointGroup.py"
  "../src/Human_intention/msg/_Pose.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
