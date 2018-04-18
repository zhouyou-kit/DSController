# add_subdirectory() directives must be maintained in order of
# least package internal library dependencies first.
# This is required to make automatic disabling of dependencies work
# (e.g. disabling a component will disable the corresponding application, too).
# `interfaces` and custom libraries come first,
# followed by `components` and finalized by `applications`.

add_subdirectory(interface)

add_subdirectory(libraries)
add_subdirectory(statecharts)