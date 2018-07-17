file(REMOVE_RECURSE
  "rviz_cloud_annotation_node_automoc.cpp"
  "rviz_cloud_annotation_plugin_automoc.cpp"
)

# Per-language clean rules from dependency scanning.
foreach(lang )
  include(CMakeFiles/visualization_msgs_generate_messages_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
endforeach()
