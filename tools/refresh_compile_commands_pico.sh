#!/usr/bin/env bash

bazel build //tplp/... --keep_going
bazel run :refresh_compile_commands_pico

# Filter out command line options unrecognized by clang.
# It errors out otherwise.
sed -i 's/^.*-mpoke-function-name.*$//' compile_commands.json
sed -i 's/^.*-pthread.*$//' compile_commands.json
