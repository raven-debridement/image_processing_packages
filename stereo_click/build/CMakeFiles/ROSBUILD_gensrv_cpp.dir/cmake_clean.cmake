FILE(REMOVE_RECURSE
  "../msg_gen"
  "../srv_gen"
  "../src/stereo_click/msg"
  "../src/stereo_click/srv"
  "../msg_gen"
  "../srv_gen"
  "CMakeFiles/ROSBUILD_gensrv_cpp"
  "../srv_gen/cpp/include/stereo_click/ConvertPoint.h"
  "../srv_gen/cpp/include/stereo_click/ConvertPoints.h"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
