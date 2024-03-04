mviz() {
    rviz -d $(rospack find cse478)/config/default.rviz
}

export UW_TOOLS_SHELL_FLAVOR=bash
@[if DEVELSPACE]@
. @(CMAKE_CURRENT_SOURCE_DIR)/config/env.sh
export SOURCES_ROOT="@(CMAKE_CURRENT_SOURCE_DIR)/src/uw_tools"

 export PATH="@(CMAKE_CURRENT_SOURCE_DIR)/scripts/available_client/:$PATH"
. "@(CMAKE_CURRENT_SOURCE_DIR)/scripts/uw_tools_client"

@[else]@
# TODO(nickswalker): Add install space support
@[end if]@