import dotenv from "dotenv";
import express, { Express, Request, Response } from "express";
import { WebSocketServer } from "ws";
import rclnodejs from "rclnodejs";
import cors from "cors";

dotenv.config();

const app: Express = express();

app.use(express.json());
app.use(cors());

app.get("/", (req: Request, res: Response) => {
  res.send("Hello World From the Typescript Server!");
});

const port = process.env.PORT || 8000;

const server = app.listen(port, () => {
  console.log(`Example app listening on port ${port}`);
});

const wss = new WebSocketServer({ server });

rclnodejs.init().then(() => {
  const node = new rclnodejs.Node("publisher_example_node");

  node.createSubscription(
    "std_msgs/msg/String",
    "/speech/raw_command",
    (msg: { data: string }) => {
      console.log(`I heard: [${msg.data}]`);
      wss.clients.forEach((client: any) => {
        if (client.readyState === 1) {
          client.send(JSON.stringify({ type: "heard", data: msg.data }));
        }
      });
    }
  );

  node.createSubscription(
    "std_msgs/msg/String",
    "/speech/text_spoken",
    (msg: { data: string }) => {
      console.log(`I said: [${msg.data}]`);
      wss.clients.forEach((client: any) => {
        if (client.readyState === 1) {
          client.send(JSON.stringify({ type: "spoken", data: msg.data }));
        }
      });
    }
  );

  // Gracefully handle SIGINT (Ctrl+C)
  process.on("SIGINT", () => {
    console.log("SIGINT received: Closing node and WebSocket server...");
    node.destroy(); // Cleanup the node
    wss.close(); // Close the WebSocket server
    process.exit(0); // Exit the process
  });

  node.spin();
});
