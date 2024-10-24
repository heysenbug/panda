#!/usr/bin/env bash
set -e

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" >/dev/null && pwd)"
cd $DIR

$DIR/install_mull.sh

GIT_REF="${GIT_REF:-origin/master}"
GIT_ROOT=$(git rev-parse --show-toplevel)
MULL_OPS="mutators: [cxx_increment, cxx_decrement, cxx_comparison, cxx_boundary, cxx_bitwise_assignment, cxx_bitwise, cxx_arithmetic_assignment, cxx_arithmetic, cxx_remove_negation]"
echo -e "$MULL_OPS" > $GIT_ROOT/mull.yml
scons --mutation -j$(nproc) -D
echo -e "timeout: 10000\ngitDiffRef: $GIT_REF\ngitProjectRoot: $GIT_ROOT" >> $GIT_ROOT/mull.yml

SAFETY_MODELS=$(find * | grep "^test_.*\.py")
for safety_model in ${SAFETY_MODELS[@]}; do
  echo ""
  echo ""
  echo -e "Testing mutations on : $safety_model"
  mull-runner-17 --ld-search-path /lib/x86_64-linux-gnu/ ../libpanda/libpanda.so -test-program=./$safety_model
done