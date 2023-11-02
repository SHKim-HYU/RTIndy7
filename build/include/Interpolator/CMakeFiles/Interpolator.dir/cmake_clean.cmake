file(REMOVE_RECURSE
  "libInterpolatord.a"
  "libInterpolatord.pdb"
)

# Per-language clean rules from dependency scanning.
foreach(lang )
  include(CMakeFiles/Interpolator.dir/cmake_clean_${lang}.cmake OPTIONAL)
endforeach()
