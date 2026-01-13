"use client";

import { Ros } from "roslib";

const createRosClient = () =>
  new Ros({
    url: "ws://localhost:9090",
  });

export const rosClient = createRosClient();
