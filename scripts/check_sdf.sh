#!/usr/bin/env bash
# Validate SDF files with `gz sdf --check`.
#
# Usage: scripts/check_sdf.sh [file.sdf ...]
# Without arguments, all tracked .sdf files are checked.
#
# `gz sdf` exits 0 even on errors, so the output is parsed instead. Unknown
# elements and attributes are reported as warnings and are treated as errors,
# except gz_frame_id which is kept for Harmonic. Fuel URIs cannot be resolved
# by `gz sdf` and are ignored.

set -u

root="$(cd "$(dirname "$0")/.." && pwd)"
export SDF_PATH="${root}/models:${root}/worlds${SDF_PATH:+:${SDF_PATH}}"
export GZ_SIM_RESOURCE_PATH="${root}/models:${root}/worlds${GZ_SIM_RESOURCE_PATH:+:${GZ_SIM_RESOURCE_PATH}}"

if ! command -v gz > /dev/null; then
  echo "check_sdf: gz not found, source your Gazebo installation" >&2
  exit 1
fi

if [ $# -eq 0 ]; then
  mapfile -t files < <(git -C "${root}" ls-files '*.sdf' | sed "s|^|${root}/|")
else
  files=("$@")
fi

status=0
for f in "${files[@]}"; do
  errors=$(gz sdf --check "${f}" 2>&1 \
    | sed 's/\x1b\[[0-9;]*m//g' \
    | grep -o -e 'Error Code [0-9]*:.*' -e 'L[0-9]*\]: XML .* not defined in SDF\.' \
    | grep -v -e 'Unable to find uri\[https://fuel\.' -e 'XML Element\[gz_frame_id\]' \
    | sort -u)
  if [ -n "${errors}" ]; then
    echo "${f}:"
    echo "${errors}"
    status=1
  fi
done
exit ${status}
