FILE(REMOVE_RECURSE
  "../msg_gen"
  "../srv_gen"
  "../msg_gen"
  "../srv_gen"
  "../src/skeleton_markers/msg"
  "../src/skeleton_markers/srv"
  "CMakeFiles/ROSBUILD_genaction_msgs"
  "../msg/recognitionActionAction.msg"
  "../msg/recognitionActionGoal.msg"
  "../msg/recognitionActionActionGoal.msg"
  "../msg/recognitionActionResult.msg"
  "../msg/recognitionActionActionResult.msg"
  "../msg/recognitionActionFeedback.msg"
  "../msg/recognitionActionActionFeedback.msg"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genaction_msgs.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
