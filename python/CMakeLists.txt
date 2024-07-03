nanobind_add_module(linefit MODULE NOMINSIZE linefit_ext.cpp)
target_link_libraries(linefit PRIVATE gm_lib)
install(TARGETS linefit LIBRARY DESTINATION .)