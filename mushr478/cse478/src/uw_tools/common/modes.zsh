#!/usr/env/zsh
###
# Custom shell modes that control the current ROS master
##

# If no mode is set, set it to none. Otherwise, keep the current mode
export UW_TOOLS_MODE=${UW_TOOLS_MODE:-"none"}

function __ros_prompt {
    UW_TOOLS_PROMPT_ROS_STATUS=$(get_ros_status)
}

function sim-mode {
    if [[ "$UW_TOOLS_MODE" == "none" ]]; then
        export original_PROMPT=$PROMPT
        export original_ROS_MASTER_URI="$ROS_MASTER_URI"
        export original_ROS_IP="$ROS_IP"
    fi

    export ROS_MASTER_URI=http://localhost:11311
    set_rosip lo
    export UW_TOOLS_MODE="sim"
    precmd_functions=("${(@)precmd_functions:#__ros_prompt}")
    precmd_functions+=__ros_prompt
    PROMPT="%{$fg[red]%}lcl \$UW_TOOLS_PROMPT_ROS_STATUS%{$reset_color%} "$original_PROMPT
}

function exit-mode {
    export ROS_MASTER_URI="$original_ROS_MASTER_URI"
    export ROS_IP="$original_ROS_IP"
    PROMPT="$original_PROMPT"
    precmd_functions=("${(@)precmd_functions:#__ros_prompt}")
    export UW_TOOLS_MODE="none"
}
