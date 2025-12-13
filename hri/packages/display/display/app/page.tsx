"use client";

import { rosClient } from "@/RosClient";
import { useState } from "react";

export default function Home() {
  const [connected, setConnected] = useState(false);

  rosClient.on("connection", () => {
    console.log("Connected to rosbridge WebSocket!");
    setConnected(true);
  });

  return <main>{connected ? "Connected to ROS" : "Not connected to ROS"}</main>;
}
