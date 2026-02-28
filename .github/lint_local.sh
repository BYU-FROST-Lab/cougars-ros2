#!/bin/bash
echo "Usage: lint_local.sh --lint-all [true or false]"
LINTER_ENV_FILE="$HOME/cougars/cougars-ros2/.github/super_linter.env"
VALIDATE_VAL="${2:-false}"
touch "$LINTER_ENV_FILE"
# If the variable exists, replace it; otherwise, append it
if grep -q "^VALIDATE_ALL_CODEBASE=" "$LINTER_ENV_FILE"; then
    sed -i "s/^VALIDATE_ALL_CODEBASE=.*/VALIDATE_ALL_CODEBASE=$VALIDATE_VAL/" "$LINTER_ENV_FILE"
else
    echo "VALIDATE_ALL_CODEBASE=$VALIDATE_VAL" >> "$LINTER_ENV_FILE"
fi

if [ -e "lint_local.sh" ]; then
    cd ..
elif [basename "$PWD" = "cougars-ros2"]; then :
else
    echo "run this script from either the top of the cougars ros2 directory or the .github directory, stopping"
    exit 1
fi

docker run --rm     -e RUN_LOCAL=true     --env-file ".github/super_linter.env"     -v $(pwd):/tmp/lint     ghcr.io/super-linter/super-linter:latest