#!/usr/bin/env bash

bazel build --config=test --build_tests_only //tplp/... --keep_going
bazel run :refresh_compile_commands_tests -- --config=test

# workaround for hedron-compile-commands-extractor oddness
# see https://github.com/hedronvision/bazel-compile-commands-extractor/issues/70
tmp=$(mktemp)
jq 'map(select(.file | (endswith(".h") | not) and (startswith("/") | not) ))' compile_commands.json >"${tmp}"
mv "${tmp}" compile_commands.json
