#!/usr/bin/env python3
"""Delete all records from RobotTrajectory DynamoDB table."""

import boto3
from botocore.exceptions import ClientError

REGION = "us-east-2"
TABLE_NAME = "RobotTrajectory"

def delete_all_records():
    """Scan and delete all items from the RobotTrajectory table."""
    dynamodb = boto3.resource('dynamodb', region_name=REGION)
    table = dynamodb.Table(TABLE_NAME)

    print(f"Scanning {TABLE_NAME} table...")

    # Scan to get all items
    scan_kwargs = {
        'ProjectionExpression': 'device_id, #ts',
        'ExpressionAttributeNames': {'#ts': 'timestamp'}
    }

    deleted_count = 0

    try:
        while True:
            response = table.scan(**scan_kwargs)
            items = response.get('Items', [])

            if not items:
                break

            # Delete items in batches
            with table.batch_writer() as batch:
                for item in items:
                    batch.delete_item(
                        Key={
                            'device_id': item['device_id'],
                            'timestamp': item['timestamp']
                        }
                    )
                    deleted_count += 1
                    if deleted_count % 100 == 0:
                        print(f"Deleted {deleted_count} records...")

            # Check if there are more items to scan
            if 'LastEvaluatedKey' not in response:
                break

            scan_kwargs['ExclusiveStartKey'] = response['LastEvaluatedKey']

        print(f"✓ Successfully deleted {deleted_count} records from {TABLE_NAME}")
        return deleted_count

    except ClientError as e:
        print(f"✗ Error: {e.response['Error']['Message']}")
        return deleted_count

if __name__ == "__main__":
    count = delete_all_records()
    print(f"\nTotal records deleted: {count}")
