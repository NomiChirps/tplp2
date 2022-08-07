#!/usr/bin/env bash

bazel build //tplp/... --keep_going
bazel run :refresh_compile_commands

# Filter out command line options unrecognized by clang.
# It errors out otherwise.
sed -i 's/^.*-mpoke-function-name.*$//' compile_commands.json