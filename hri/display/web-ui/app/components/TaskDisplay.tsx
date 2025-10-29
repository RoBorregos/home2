"use client";

import React from "react";
import GPSRDisplay from "./GPSRDisplay";
import StoreGroceriesDisplay from "./StoreGroceriesDisplay";

type Props = {
  task: string;
};

export default function TaskDisplay({ task }: Props): React.ReactElement | null {
  const t = String(task ?? "").trim().toLowerCase();

  if (t.includes("gpsr")) {
    return <GPSRDisplay />;
  }

  if (
    t.includes("store") ||
    t.includes("grocery") ||
    t.includes("groceries") ||
    t.includes("storegroceries")
  ) {
    return <StoreGroceriesDisplay />;
  }

  return null;
}