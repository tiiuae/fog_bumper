#!/bin/bash

set -euxo pipefail

output_dir=$1

build_number=${GITHUB_RUN_NUMBER:=0}

iname=fog-bumper

docker build \
  --build-arg UID=$(id -u) \
  --build-arg GID=$(id -g) \
  --pull \
  -f Dockerfile -t "${iname}:latest" .

docker run \
  --rm \
  -v $(pwd):/fog_bumper/sources \
  fog-bumper:latest \
  ./packaging/package.sh \
  -b ${build_number}

mkdir -p ${output_dir}
cp *.deb *.ddeb ${output_dir}
rm -Rf *.deb *.ddeb

exit 0
