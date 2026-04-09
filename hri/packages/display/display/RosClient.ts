"use client";

import { Ros } from "roslib";

const createRosClient = () => {
  const url = "ws://localhost:9090";
  let ros = new Ros({ url: url });
  let reconnectTimer: NodeJS.Timeout | null = null;

  const attemptConnection = () => {
    if (ros.isConnected) return;
    try {
      console.log("Attempting to reconnect to ROS bridge...");
      ros.connect(url);
    } catch (error) {
      console.error("Failed handling ROS connection attempt:", error);
    }
  };

  const setupEvents = () => {
    ros.on("close", () => {
      console.warn("ROS connection closed");
      if (!reconnectTimer) {
        reconnectTimer = setInterval(attemptConnection, 3000);
      }
    });

    ros.on("error", (error) => {
      console.error("ROS connection error:", error);
      if (!reconnectTimer) {
        reconnectTimer = setInterval(attemptConnection, 3000);
      }
    });

    ros.on("connection", () => {
      console.log("ROS Connected!");
      if (reconnectTimer) {
        clearInterval(reconnectTimer);
        reconnectTimer = null;
      }
    });
  };

  setupEvents();

  return ros;
};

export const rosClient = createRosClient();


