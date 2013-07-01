FILE(REMOVE_RECURSE
  "../msg_gen"
  "../srv_gen"
  "../src/stereo_click/msg"
  "../src/stereo_click/srv"
  "../msg_gen"
  "../srv_gen"
  "CMakeFiles/ROSBUILD_gensrv_lisp"
  "../srv_gen/lisp/ConvertPoint.lisp"
  "../srv_gen/lisp/_package.lisp"
  "../srv_gen/lisp/_package_ConvertPoint.lisp"
  "../srv_gen/lisp/ConvertPoints.lisp"
  "../srv_gen/lisp/_package.lisp"
  "../srv_gen/lisp/_package_ConvertPoints.lisp"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_lisp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
