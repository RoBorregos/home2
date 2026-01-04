"use client";

import { useEffect, useState } from "react";
import { rosClient } from "../RosClient";

export function ConnectionStatus() {
  const [connected, setConnected] = useState(false);

  useEffect(() => {
    const handleConnection = () => setConnected(true);
    const handleDisconnection = () => setConnected(false);
    const handleError = () => setConnected(false);

    rosClient.on("connection", handleConnection);
    rosClient.on("close", handleDisconnection);
    rosClient.on("error", handleError);

    // Check initial connection state
    if (rosClient.isConnected) {
      setConnected(true);
    }

    return () => {
      rosClient.off("connection", handleConnection);
      rosClient.off("close", handleDisconnection);
      rosClient.off("error", handleError);
    };
  }, []);

  return (
    <div
      className={
        connected
          ? "px-2 py-1 rounded-full text-xs font-medium bg-(--blue-bg) text-(--blue)"
          : "px-2 py-1 rounded-full text-xs font-medium bg-(--orange-bg) text-(--orange)"
      }
    >
      {connected ? "Connected" : "Disconnected"}
    </div>
  );
}
