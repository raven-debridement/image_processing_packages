FILE(REMOVE_RECURSE
  "../msg_gen"
  "../srv_gen"
  "../src/stereo_click/msg"
  "../src/stereo_click/srv"
  "../msg_gen"
  "../srv_gen"
  "CMakeFiles/ROSBUILD_gensrv_py"
  "../src/stereo_click/srv/__init__.py"
  "../src/stereo_click/srv/_ConvertPoint.py"
  "../src/stereo_click/srv/_ConvertPoints.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
