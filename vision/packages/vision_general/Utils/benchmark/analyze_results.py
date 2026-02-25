#!/usr/bin/env python3
"""
Analyze and visualize MOT benchmark results
"""

import pandas as pd
import os


def main():
    csv_path = "./output/results/summary.csv"

    if not os.path.exists(csv_path):
        return

    df = pd.read_csv(csv_path)

    try:
        import matplotlib.pyplot  # noqa: F401
    except ImportError:
        return

    create_visualizations(df)


def create_visualizations(df):
    """Create bar charts for key metrics"""
    import matplotlib.pyplot as plt

    df_sorted = df.sort_values("mota", ascending=False)

    fig, axes = plt.subplots(2, 2, figsize=(15, 10))
    fig.suptitle("MOT16 Tracker Benchmark Results", fontsize=16, fontweight="bold")

    # MOTA
    axes[0, 0].barh(df_sorted["tracker"], df_sorted["mota"] * 100, color="#2ecc71")
    axes[0, 0].set_xlabel("MOTA (%)")
    axes[0, 0].set_title("Multiple Object Tracking Accuracy (Higher = Better)")
    axes[0, 0].grid(axis="x", alpha=0.3)

    # IDF1
    axes[0, 1].barh(df_sorted["tracker"], df_sorted["idf1"] * 100, color="#3498db")
    axes[0, 1].set_xlabel("IDF1 (%)")
    axes[0, 1].set_title("ID F1 Score (Higher = Better)")
    axes[0, 1].grid(axis="x", alpha=0.3)

    # ID Switches
    switches_sorted = df.sort_values("num_switches", ascending=True)
    axes[1, 0].barh(
        switches_sorted["tracker"], switches_sorted["num_switches"], color="#e74c3c"
    )
    axes[1, 0].set_xlabel("Number of Switches")
    axes[1, 0].set_title("ID Switches (Lower = Better)")
    axes[1, 0].grid(axis="x", alpha=0.3)

    # Recall
    axes[1, 1].barh(df_sorted["tracker"], df_sorted["recall"] * 100, color="#f39c12")
    axes[1, 1].set_xlabel("Recall (%)")
    axes[1, 1].set_title("Detection Recall (Higher = Better)")
    axes[1, 1].grid(axis="x", alpha=0.3)

    plt.tight_layout()
    plt.show()
    plt.close()


if __name__ == "__main__":
    main()
