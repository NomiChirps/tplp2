#!/usr/bin/env bash

bazel build --config=test --build_tests_only //tplp/... --keep_going
bazel run :refresh_compile_commands_tests -- --config=test
