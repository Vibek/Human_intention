FILE(REMOVE_RECURSE
  "../msg_gen"
  "../msg_gen"
  "../src/Human_intention/msg"
  "CMakeFiles/ROSBUILD_genmsg_cpp"
  "../msg_gen/cpp/include/Human_intention/Skeleton.h"
  "../msg_gen/cpp/include/Human_intention/PerceptInfo.h"
  "../msg_gen/cpp/include/Human_intention/Goal.h"
  "../msg_gen/cpp/include/Human_intention/EnableJointGroup.h"
  "../msg_gen/cpp/include/Human_intention/Pose.h"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
