#!/bin/bash

echo "Cleaning workspace artifacts..."
rm -rf build/ install/ log/

echo "Cleaning independent cargo artifacts..."
# Find all Cargo.toml files in the src directory
find src -name "Cargo.toml" | while read cargo_file; do
    dir=$(dirname "$cargo_file")
    if [ -d "$dir/target" ]; then
        echo "Removing target in $dir"
        rm -rf "$dir/target"
    fi
done

echo "Clean complete."
