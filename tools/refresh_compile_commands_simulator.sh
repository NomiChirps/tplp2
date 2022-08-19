#!/usr/bin/env bash

bazel build --config=simulate //simulator/... --keep_going
bazel run //simulator:refresh_compile_commands -- --config=simulate

# workaround for hedron-compile-commands-extractor oddness
# see https://github.com/hedronvision/bazel-compile-commands-extractor/issues/70
tmp=$(mktemp)
jq 'map(select(.file | (endswith(".h") | not) and (startswith("/") | not) ))' compile_commands.json >"${tmp}"
mv "${tmp}" compile_commands.json
