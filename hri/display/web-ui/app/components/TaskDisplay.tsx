"use client";

import React, { JSX } from "react";
import GPSRDisplay from "./GPSRDisplay";
import StoreGroceriesDisplay from "./StoreGroceriesDisplay";

type Props = {
  task: string;
};

export default function TaskDisplay({ task }: Props): JSX.Element | null {
  if (task === "GPSR") return <GPSRDisplay />;
  if (task === "StoreGroceries") return <StoreGroceriesDisplay />;
  return null;
}