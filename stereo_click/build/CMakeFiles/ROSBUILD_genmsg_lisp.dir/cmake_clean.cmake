FILE(REMOVE_RECURSE
  "../msg_gen"
  "../srv_gen"
  "../src/stereo_click/msg"
  "../src/stereo_click/srv"
  "../msg_gen"
  "../srv_gen"
  "CMakeFiles/ROSBUILD_genmsg_lisp"
  "../msg_gen/lisp/ClickPoint.lisp"
  "../msg_gen/lisp/_package.lisp"
  "../msg_gen/lisp/_package_ClickPoint.lisp"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_lisp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
