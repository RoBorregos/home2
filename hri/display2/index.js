'use strict';

const rclnodejs = require('rclnodejs');
rclnodejs.init().then(() => {
  const node = new rclnodejs.Node('publisher_example_node');
  const sttSubscriber = node.createSubscription('std_msgs/msg/String', '/speech/raw_command', (msg) => {
    console.log(`I heard: [${msg.data}]`);
  });
  const ttsSubscriber = node.createSubscription('std_msgs/msg/String', '/speech/text_spoken', (msg) => {
    console.log(`I said: [${msg.data}]`);
  });
  node.spin();
});