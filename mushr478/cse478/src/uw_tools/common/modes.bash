#!/usr/bin/env bash
###
# Custom shell modes that control the current ROS master
##

# If no mode is set, set it to none. Otherwise, keep the current mode
export UW_TOOLS_MODE=${UW_TOOLS_MODE:-"none"}

function __prompt_command {
    UW_TOOLS_PROMPT_ROS_STATUS=$(get_ros_status)
}

function sim-mode {
    if [[ "$UW_TOOLS_MODE" == "none" ]]; then
        export original_PROMPT_COMMAND="$PROMPT_COMMAND"
        export original_PS1="$PS1"
        export original_ROS_MASTER_URI="$ROS_MASTER_URI"
        export original_ROS_IP="$ROS_IP"
    fi

    export ROS_MASTER_URI=http://localhost:11311
    set_rosip lo
    export UW_TOOLS_MODE="sim"
    PROMPT_COMMAND=__prompt_command
    PS1="\[\033[41;1;37m\]lcl \$UW_TOOLS_PROMPT_ROS_STATUS\[\033[0m\]:\[\033[01;34m\]\w\[\033[00m\]\$ "
}

function exit-mode {
    export ROS_MASTER_URI="$original_ROS_MASTER_URI"
    export ROS_IP="$original_ROS_IP"
    PS1="$original_PS1"
    PROMPT_COMMAND="$original_PROMPT_COMMAND"
    export UW_TOOLS_MODE="none"
}
