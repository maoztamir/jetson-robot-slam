#!/usr/bin/env bash
#
# Deploy the real Lambda code to replace the CloudFormation placeholder.
#
# Usage:
#   ./scripts/deploy_lambda.sh
#
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

FUNCTION_NAME="jetson-robot-slam-process-trajectory"
LAMBDA_SRC="$PROJECT_ROOT/aws/lambda/process_trajectory.py"
TMPDIR="${TMPDIR:-/tmp}"
ZIP_FILE="$TMPDIR/process_trajectory.zip"

echo "=== Deploy Lambda: $FUNCTION_NAME ==="

# Verify source exists
if [[ ! -f "$LAMBDA_SRC" ]]; then
  echo "ERROR: Lambda source not found at $LAMBDA_SRC" >&2
  exit 1
fi

# Package
echo "Packaging $LAMBDA_SRC -> $ZIP_FILE"
cd "$(dirname "$LAMBDA_SRC")"
zip -j "$ZIP_FILE" "$(basename "$LAMBDA_SRC")"

# Deploy
echo "Updating function code..."
aws lambda update-function-code \
  --function-name "$FUNCTION_NAME" \
  --zip-file "fileb://$ZIP_FILE" \
  --no-cli-pager

echo ""
echo "Verifying..."
aws lambda get-function-configuration \
  --function-name "$FUNCTION_NAME" \
  --query '{FunctionName: FunctionName, Runtime: Runtime, Handler: Handler, LastModified: LastModified, CodeSize: CodeSize}' \
  --output table \
  --no-cli-pager

# Clean up
rm -f "$ZIP_FILE"

echo ""
echo "Lambda deployed successfully."
