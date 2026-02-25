#!/usr/bin/env python3
"""Count RobotTrajectory records in DynamoDB, grouped by device_id."""

import boto3
from collections import defaultdict

TABLE  = "RobotTrajectory"
REGION = "us-east-2"


def main():
    table = boto3.resource("dynamodb", region_name=REGION).Table(TABLE)

    counts = defaultdict(int)
    scanned = 0
    kwargs = {
        "ProjectionExpression": "device_id",
        "Select": "SPECIFIC_ATTRIBUTES",
    }

    while True:
        resp = table.scan(**kwargs)
        for item in resp["Items"]:
            counts[item["device_id"]] += 1
        scanned += resp["ScannedCount"]

        if "LastEvaluatedKey" not in resp:
            break
        kwargs["ExclusiveStartKey"] = resp["LastEvaluatedKey"]

    print(f"{'Device ID':<30}  {'Count':>8}")
    print("-" * 42)
    for device_id, count in sorted(counts.items()):
        print(f"{device_id:<30}  {count:>8,}")
    print("-" * 42)
    print(f"{'TOTAL':<30}  {sum(counts.values()):>8,}")
    print(f"\n(scanned {scanned:,} items)")


if __name__ == "__main__":
    main()
