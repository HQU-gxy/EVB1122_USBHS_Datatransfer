#!/usr/bin/env bash
rg --files -t cpp -t c | xargs clang-format -i