file(REMOVE_RECURSE
  "libRobot.a"
  "libRobot.pdb"
)

# Per-language clean rules from dependency scanning.
foreach(lang )
  include(CMakeFiles/Robot.dir/cmake_clean_${lang}.cmake OPTIONAL)
endforeach()
