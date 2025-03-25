import express from 'express';
import { WebSocketServer } from 'ws';
import rclnodejs from 'rclnodejs';

const app = express();
const server = app.listen(3001, () =>
  console.log('Server listening on port 3001'),
);
const wss = new WebSocketServer({ server });

// Serve the React app
app.use(express.static('public'));

rclnodejs.init().then(() => {
  const node = new rclnodejs.Node('publisher_example_node');

  node.createSubscription(
    'std_msgs/msg/String',
    '/speech/raw_command',
    (msg: { data: string }) => {
      console.log(`I heard: [${msg.data}]`);
      wss.clients.forEach((client: any) => {
        if (client.readyState === 1) {
          client.send(JSON.stringify({ type: 'heard', data: msg.data }));
        }
      });
    },
  );

  node.createSubscription(
    'std_msgs/msg/String',
    '/speech/text_spoken',
    (msg: { data: string }) => {
      console.log(`I said: [${msg.data}]`);
      wss.clients.forEach((client: any) => {
        if (client.readyState === 1) {
          client.send(JSON.stringify({ type: 'spoken', data: msg.data }));
        }
      });
    },
  );

  node.spin();
});
