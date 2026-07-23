#!/bin/bash
set -e

cd "$(dirname "$0")"  # ensures it runs from the script's own directory

# Use provided commit message, or fall back to a timestamp
if [ -n "$1" ]; then
    MSG="$*"
else
    MSG="Update $(date '+%Y-%m-%d %H:%M:%S')"
fi

echo "== Git auto push =="
echo "Working directory: $(pwd)"

# Show what will be committed
git status --short

git add -A

# Only commit if there's actually something staged
if git diff --cached --quiet; then
    echo "Nothing to commit."
else
    git commit -m "$MSG"
fi

echo "Pushing to remote..."
git push

echo "Done."