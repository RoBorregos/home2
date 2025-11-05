#!/bin/bash

_run_sh_autocomplete() {
    local cur prev words cword
    _init_completion || return

    local areas="manipulation navigation hri vision integration"
    local tasks="--carry --receptionist --storing-groceries --gpsr  --gpsr-display --groceries-display --moondream --egpsr --clean-table --hand --restaurant"
    local flags="--rebuild --help -h -d"
    
    # Add area specific flags
    case "${words[1]}" in
        hri)
            flags="$flags --build-display --open-display --download-model --open-groceries-display --open-gpsr-display --display=GPSR --display=StoreGroceries"
            ;;
        integration)
            flags="$flags --build --test-hri"
    esac

    local options
    case ${COMP_CWORD} in
        1)
            options="$areas"
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
