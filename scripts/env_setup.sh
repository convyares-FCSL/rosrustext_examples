#!/bin/bash

# Helper to suppress warnings about missing paths in AMENT/CMAKE_PREFIX_PATH
filter_missing_paths() {
    local var_name="$1"
    local raw_paths="${!var_name}"
    local clean_paths=""

    if [ -n "$raw_paths" ]; then
        IFS=':' read -ra ADDR <<< "$raw_paths"
        for path in "${ADDR[@]}"; do
            if [ -d "$path" ]; then
                if [ -z "$clean_paths" ]; then
                    clean_paths="$path"
                else
                    clean_paths="${clean_paths}:${path}"
                fi
            fi
        done
        export $var_name="$clean_paths"
    fi
}

sanitize_environment() {
    echo "Sanitizing environment..."
    filter_missing_paths AMENT_PREFIX_PATH
    filter_missing_paths CMAKE_PREFIX_PATH
}
