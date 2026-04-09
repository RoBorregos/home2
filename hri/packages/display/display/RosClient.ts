"use client";

import { Ros } from "roslib";

const createRosClient = () => {
  const url = "ws://localhost:9090";
  const ros = new Ros({
    url: url,
  });

  const attemptConnection = () => {
    if (ros.isConnected) return;
    try {
      ros.connect(url);
    } catch (error) {
    }
  };

  ros.on("close", () => {
    setTimeout(attemptConnection, 3000);
  });


  return ros;
};

export const rosClient = createRosClient();


