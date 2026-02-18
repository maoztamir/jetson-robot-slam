#!/usr/bin/env bash
#
# Deploy the web dashboard to an S3 static website bucket.
#
# Usage:
#   ./scripts/deploy_web.sh                    # uses default bucket name
#   ./scripts/deploy_web.sh my-bucket-name     # custom bucket name
#
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

BUCKET_NAME="${1:-jetson-robot-slam-dashboard}"
WEB_DIR="$PROJECT_ROOT/web"
REGION="us-east-2"

echo "=== Deploy Web Dashboard ==="
echo "Bucket : $BUCKET_NAME"
echo "Region : $REGION"
echo "Source : $WEB_DIR"
echo ""

# Verify web directory exists
if [[ ! -d "$WEB_DIR" ]]; then
  echo "ERROR: Web directory not found at $WEB_DIR" >&2
  exit 1
fi

# Create bucket if it doesn't exist
if ! aws s3api head-bucket --bucket "$BUCKET_NAME" 2>/dev/null; then
  echo "Creating S3 bucket: $BUCKET_NAME"
  aws s3api create-bucket \
    --bucket "$BUCKET_NAME" \
    --region "$REGION" \
    --create-bucket-configuration LocationConstraint="$REGION" \
    --no-cli-pager
fi

# Disable block public access for static website hosting
echo "Configuring public access..."
aws s3api put-public-access-block \
  --bucket "$BUCKET_NAME" \
  --public-access-block-configuration \
  'BlockPublicAcls=false,IgnorePublicAcls=false,BlockPublicPolicy=false,RestrictPublicBuckets=false' \
  --no-cli-pager

# Enable static website hosting
echo "Enabling static website hosting..."
aws s3 website "s3://$BUCKET_NAME" \
  --index-document index.html \
  --error-document index.html

# Set bucket policy for public read
echo "Setting bucket policy..."
cat <<POLICY | aws s3api put-bucket-policy --bucket "$BUCKET_NAME" --policy file:///dev/stdin --no-cli-pager
{
  "Version": "2012-10-17",
  "Statement": [
    {
      "Sid": "PublicReadGetObject",
      "Effect": "Allow",
      "Principal": "*",
      "Action": "s3:GetObject",
      "Resource": "arn:aws:s3:::${BUCKET_NAME}/*"
    }
  ]
}
POLICY

# Upload files with correct content types
echo "Uploading files..."
aws s3 sync "$WEB_DIR" "s3://$BUCKET_NAME/" \
  --delete \
  --cache-control "max-age=300" \
  --no-cli-pager

# Set correct content types
aws s3 cp "s3://$BUCKET_NAME/" "s3://$BUCKET_NAME/" \
  --recursive \
  --exclude "*" \
  --include "*.html" \
  --content-type "text/html" \
  --metadata-directive REPLACE \
  --cache-control "max-age=300" \
  --no-cli-pager

aws s3 cp "s3://$BUCKET_NAME/" "s3://$BUCKET_NAME/" \
  --recursive \
  --exclude "*" \
  --include "*.css" \
  --content-type "text/css" \
  --metadata-directive REPLACE \
  --cache-control "max-age=300" \
  --no-cli-pager

aws s3 cp "s3://$BUCKET_NAME/" "s3://$BUCKET_NAME/" \
  --recursive \
  --exclude "*" \
  --include "*.js" \
  --content-type "application/javascript" \
  --metadata-directive REPLACE \
  --cache-control "max-age=300" \
  --no-cli-pager

# Output website URL
WEBSITE_URL="http://${BUCKET_NAME}.s3-website.${REGION}.amazonaws.com"
echo ""
echo "============================================"
echo "Dashboard deployed successfully!"
echo ""
echo "URL: $WEBSITE_URL"
echo "============================================"
echo ""
echo "IMPORTANT: Update web/js/app.js CONFIG with:"
echo "  - identityPoolId: your Cognito Identity Pool ID"
echo "  - iotEndpoint: your IoT data endpoint"
echo ""
echo "Then re-run this script to upload the updated config."
