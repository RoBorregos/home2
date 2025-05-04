#!/bin/bash

_run_sh_autocomplete() {
    local cur prev words cword
    _init_completion || return

    local areas="manipulation navigation hri vision integration"
    local tasks="--carry --receptionist --storing-groceries --gpsr --moondream"
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
