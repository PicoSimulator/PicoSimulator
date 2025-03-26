#!/usr/bin/env bash

#####################################
#
# Bash autocompletion script
#
# Ensure this file gets sourced and 
# run to enable tab-completion.
#
#####################################

picosim_app_run_complete()
{
    COMPREPLY+=($(compgen -W "$(${COMP_WORDS[0]} list -e -q)" "${COMP_WORDS[2]}"))
}

picosim_app_complete()
{
    
    if [ "${COMP_WORDS[1]}" == "run" ]
    then
        picosim_app_run_complete
    fi
    if [ "${#COMP_WORDS[@]}" != "2" ]
    then
        return
    fi
    COMPREPLY+=($(compgen -W "run list" "${COMP_WORDS[1]}"))
}

complete -F picosim_app_complete picosim_app