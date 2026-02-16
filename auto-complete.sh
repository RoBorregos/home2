#!/bin/bash

_run_sh_autocomplete() {
    local cur words
    _init_completion || return

    local areas="manipulation navigation hri vision integration frida_interfaces"
    local inputs="--stop --down"
    local tasks="--hric --ppc --gpsr --dlc --restaurant --finals"
    local flags="--build --build-image --recreate --down --stop --help -h -d"
    
    # Add area specific flags
    case "${words[1]}" in
        hri)
            flags="$flags --build-display --open-display --download-model"
            ;;
        integration)
            # TODO: add other important scripts
            flags="$flags --test-hri"
    esac

    local options
    case ${COMP_CWORD} in
        1)
            options="$areas $inputs $tasks"
            ;;
        2)
            options="$tasks"
            ;;
        *)  
            options=""
            ;;
    esac
    
    COMPREPLY=( $(compgen -W "$options $flags" -- "$cur") )
}

complete -F _run_sh_autocomplete ./run.sh
complete -F _run_sh_autocomplete ./status.sh

# To use this script:
# Source this script in your terminal: `source auto-complete.sh` or 
# add it to your shell's configuration file (e.g., ~/.bashrc or ~/.bash_profile).
