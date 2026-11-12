# Source local env files. Intended to be sourced from start_*.sh, not executed.
_dimos_load_env() {
  local file
  for file in "$@"; do
    if [[ -f "$file" ]]; then
      set -a
      # shellcheck disable=SC1090
      source "$file"
      set +a
    fi
  done
}
