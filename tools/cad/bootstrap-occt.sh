#!/usr/bin/env bash
# Ubuntu 22.04: download/extract OCCT locally; never perform a system install.
set -euo pipefail
cad_repo_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)
cad_prefix="$cad_repo_root/.deps/occt"
mkdir -p "$cad_prefix/downloads"
cd "$cad_prefix/downloads"
apt-get download \
  libocct-data-exchange-dev libocct-data-exchange-7.5 \
  libocct-modeling-algorithms-dev libocct-modeling-algorithms-7.5 \
  libocct-modeling-data-dev libocct-modeling-data-7.5 \
  libocct-foundation-dev libocct-foundation-7.5 libtbb2 libtbbmalloc2
for cad_package in ./*.deb; do
  dpkg-deb -x "$cad_package" "$cad_prefix"
done
sha256sum ./*.deb > "$cad_prefix/package-sha256.txt"
for cad_package in ./*.deb; do
  dpkg-deb -f "$cad_package" Package Version Architecture
done > "$cad_prefix/package-versions.txt"
