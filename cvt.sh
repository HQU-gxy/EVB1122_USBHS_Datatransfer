#!/bin/bash

# Loop through all .c, .cpp, and .h files
find ./ -name "*.c" -o -name "*.cpp" -o -name "*.h" -type f | while read -r file; do
    # Detect the encoding using chardet
    encoding=$(python -m chardet "$file" | awk '{print $2}')

    # Convert the file to UTF-8 using iconv
    if [[ "$encoding" != "utf-8" && \
          "$encoding" != "ascii"    ]]; then
        iconv -f "$encoding" -t utf-8 "$file" -o "${file}.utf8"
        mv "${file}.utf8" "$file"
        echo "Converted $file from $encoding to UTF-8"
    fi
done


