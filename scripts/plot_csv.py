#!/usr/bin/env python3
"""Plot every column of a CSV file against its first column."""

import csv
import sys

import matplotlib.pyplot as plt


def main():
    if len(sys.argv) != 2:
        print(f"usage: {sys.argv[0]} <path/to/file.csv>")
        sys.exit(1)

    with open(sys.argv[1], newline="") as f:
        reader = csv.reader(f)
        header = next(reader)
        rows = [[float(value) for value in row] for row in reader]

    x = [row[0] for row in rows]
    for col_index, col_name in enumerate(header[1:], start=1):
        y = [row[col_index] for row in rows]
        plt.plot(x, y, label=col_name)

    plt.xlabel(header[0])
    plt.legend()
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
