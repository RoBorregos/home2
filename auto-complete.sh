#!/bin/bash

_run_sh_autocomplete() {
    local cur words
    _init_completion || return

    local areas="manipulation navigation hri vision integration frida_interfaces"
    local control="--stop --down"
    local tasks="--receptionist --storing-groceries --gpsr --egpsr --restaurant"
    local flags="--build --build-image --recreate -d"
    local help="--help -h"
    
    # Add area specific flags
    local hri="--build-display --open-display --download-model"
    local integration="--test-hri"
    case "${words[1]}" in
        hri)
            flags="$flags $hri"
            ;;
        integration)
            flags="$flags $integration"
            ;;
        --receptionist|--storing-groceries|--gpsr|--egpsr|--restaurant)
            flags="$flags $hri $integration"
            ;;
        *)
            flags=""
            ;;
    esac

    local options
    case ${COMP_CWORD} in
        1)
            options="$areas $control $tasks $help"
            ;;
        2)
            if [[ " $areas " == *" ${words[1]} "* ]]; then
                options="$control $tasks"
            fi
            ;;
        3)
            if [[ " $control " == *" ${words[2]} "* ]]; then
                flags=""
            fi
            ;;
    esac
    
    # Use mapfile to safely read compgen output into the array without word-splitting
    mapfile -t COMPREPLY < <(compgen -W "$options $flags" -- "$cur")
}

complete -F _run_sh_autocomplete ./run.sh
complete -F _run_sh_autocomplete ./status.sh

# To use this script:
# Source this script in your terminal: `source auto-complete.sh` or 
# add it to your shell's configuration file (e.g., ~/.bashrc or ~/.bash_profile).
