if [ -f /etc/bash.bashrc ]; then
    source /etc/bash.bashrc
fi

export TERM=xterm-256color

if [ -f /etc/bash_completion ]; then
    source /etc/bash_completion
fi

source /opt/ros/noetic/setup.bash

if [ -f "~/catkin_ws/devel/setup.bash" ]; then
    echo "Sourcing workspace: ~/catkin_ws/devel/setup.bash"
    source "~/catkin_ws/devel/setup.bash"
fi

export HISTCONTROL=ignoredups:erasedups
export HISTSIZE=50000
export SAVEHIST=50000
shopt -s histappend
PROMPT_COMMAND="history -a; $PROMPT_COMMAND"

export PS1="\[\e[36m\]\u@ros-noetic:\w\[\e[0m\]\$ "
