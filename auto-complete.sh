#!/bin/bash

_run_sh_autocomplete() {
    local cur prev words cword
    _init_completion || return

    local areas="manipulation navigation hri vision integration interfaces zed"
    local tasks="--carry --receptionist --storing-groceries --gpsr --moondream --egpsr --clean-table --hand --restaurant"
    local flags="--rebuild --help -h -d"

    case ${COMP_CWORD} in
        1)
            COMPREPLY=( $(compgen -W "$areas $flags" -- "$cur") )
            ;;
        2)
            COMPREPLY=( $(compgen -W "$tasks $flags" -- "$cur") )
            ;;
        *)
            COMPREPLY=( $(compgen -W "$flags" -- "$cur") )
            ;;
    esac
}

complete -F _run_sh_autocomplete ./run.sh
complete -F _run_sh_autocomplete ./status.sh

# To use this script:
# Source this script in your terminal: `source auto-complete.sh` or 
# add it to your shell's configuration file (e.g., ~/.bashrc or ~/.bash_profile).
