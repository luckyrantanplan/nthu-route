#!/bin/bash
# Script to run clang-tidy on all C++ files (excluding third-party libraries)
# Usage: ./run-clang-tidy.sh [--fix]

set -e

# Check if build directory exists
if [ ! -d "build" ]; then
    echo "Error: build directory not found. Run 'cmake -B build' first."
    exit 1
fi

# Check if compile_commands.json exists
if [ ! -f "build/compile_commands.json" ]; then
    echo "Error: compile_commands.json not found. Build the project first."
    exit 1
fi

# Check if --fix flag is provided
FIX_FLAG=""
if [ "$1" == "--fix" ]; then
    FIX_FLAG="--fix"
    echo "Running clang-tidy with automatic fixes..."
else
    echo "Running clang-tidy in check mode (use --fix to apply fixes)..."
fi

# Find all C++ files excluding third-party libraries
FILES=$(find src tests -type f \( -name "*.cpp" -o -name "*.h" \) ! -path "*/spdlog/*" | sort)

# Count total files
TOTAL=$(echo "$FILES" | wc -l)
CURRENT=0

echo "Processing $TOTAL files..."
echo ""

# Run clang-tidy on each file
for file in $FILES; do
    CURRENT=$((CURRENT + 1))
    echo "[$CURRENT/$TOTAL] Processing $file..."
    
    # Run clang-tidy
    clang-tidy -p build $FIX_FLAG "$file" 2>&1 | grep -v "^[0-9]* warnings generated" || true
done

echo ""
echo "Done! Processed $TOTAL files."
