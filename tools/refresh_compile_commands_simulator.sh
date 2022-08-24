#!/usr/bin/env bash

bazel build --config=simulate //simulator/... --keep_going
bazel run //simulator:refresh_compile_commands -- --config=simulate
