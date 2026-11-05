#!/usr/bin/env bash
# Shallow-fetch one arduino-esp32 commit and check it out as <branch>, so that
# ci-arduino's build_platform.py reuses it instead of cloning.
# Usage: fetch_bsp.sh <owner> <branch> <sha> <dest>
set -euo pipefail
owner=$1 branch=$2 sha=$3 dest=$4

rm -rf "$dest"
mkdir -p "$dest"
cd "$dest"
git init -q
git remote add origin "https://github.com/${owner}/arduino-esp32.git"
for attempt in 1 2 3 4; do
  if timeout 600 git fetch -q --depth 1 origin "$sha"; then
    break
  fi
  [ "$attempt" -eq 4 ] && { echo "::error::could not fetch ${owner}/arduino-esp32@${sha}"; exit 1; }
  echo "::warning::BSP fetch attempt $attempt failed, retrying"
  sleep $((attempt * 30))
done
git checkout -q -B "$branch" FETCH_HEAD
echo "Fetched ${owner}/arduino-esp32 ${branch} @ ${sha}"
