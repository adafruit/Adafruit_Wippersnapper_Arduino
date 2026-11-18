# Source from a workflow step: `source .github/scripts/retry.sh`
#
# clone_retry <git clone args...> <dest>
#   git clone with up to 4 attempts and backoff. The partial destination is
#   removed between attempts, because git refuses to clone into a non-empty dir.
clone_retry() {
  local dest="${!#}" attempt
  for attempt in 1 2 3 4; do
    rm -rf "$dest"
    if timeout 600 git clone "$@"; then
      return 0
    fi
    echo "::warning::git clone into $dest failed (attempt $attempt)"
    [ "$attempt" -lt 4 ] && sleep $((attempt * 20))
  done
  echo "::error::git clone into $dest failed after 4 attempts"
  return 1
}
