"use strict";

import { NextApiRequest, NextApiResponse } from "next";
import * as rclnodejs from "rclnodejs";

export default async function handler(
  req: NextApiRequest,
  res: NextApiResponse
) {
  rclnodejs.init().then(() => {
    console.log("ROS2 node initialized");
    const node = new rclnodejs.Node("publisher_example_node");
    const sttSubscriber = node.createSubscription(
      "std_msgs/msg/String",
      "/speech/raw_command",
      (msg) => {
        console.log(`I heard: [${msg.data}]`);
      }
    );
    const ttsSubscriber = node.createSubscription(
      "std_msgs/msg/String",
      "/speech/text_spoken",
      (msg) => {
        console.log(`I said: [${msg.data}]`);
      }
    );
    node.spin();
  });
}
