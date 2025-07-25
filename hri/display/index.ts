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

let publisher: rclnodejs.Publisher<"std_msgs/msg/Empty"> | null = null;
app.get("/send_button_press", (req: Request, res: Response) => {
  if (!publisher) {
    res.status(500).send("Publisher not initialized");
    return;
  }
  try {
    publisher.publish({});
    res.status(200).send("Button press message sent");
  } catch (error) {
    console.error("Error sending button press message:", error);
    res.status(500).send("Failed to send button press message");
  }
});


const port = 8001;

const server = app.listen(port, () => {
  console.log(`ROS2 websocket running on port ${port}`);
});

const wss = new WebSocketServer({ server });

rclnodejs.init().then(() => {
  const node = new rclnodejs.Node("publisher_example_node");

  node.createSubscription(
    "std_msgs/msg/String",
    "/speech/raw_command",
    (msg: { data: string }) => {
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
      wss.clients.forEach((client: any) => {
        if (client.readyState === 1) {
          client.send(JSON.stringify({ type: "spoken", data: msg.data }));
        }
      });
    }
  );

  node.createSubscription(
    "std_msgs/msg/String",
    "/hri/speech/oww",
    (msg: { data: string }) => {
      wss.clients.forEach((client: any) => {
        if (client.readyState === 1) {
          client.send(JSON.stringify({ type: "keyword", data: msg.data }));
        }
      });
    }
  );

  node.createSubscription(
    "std_msgs/msg/String",
    "/AudioState",
    (msg: { data: string }) => {
      wss.clients.forEach((client: any) => {
        if (client.readyState === 1) {
          client.send(JSON.stringify({ type: "audioState", data: msg.data }));
        }
      });
    }
  );

  node.createSubscription(
    "std_msgs/msg/Float32",
    "/hri/speech/vad",
    (msg: { data: number }) => {
      wss.clients.forEach((client: any) => {
        if (client.readyState === 1) {
          client.send(JSON.stringify({ type: "vad", data: msg.data }));
        }
      });
    }
  );

  node.createSubscription(
    "std_msgs/msg/String",
    "/hri/display/change_video",
    (msg: { data: string }) => {
      wss.clients.forEach((client: any) => {
        if (client.readyState === 1) {
          client.send(JSON.stringify({ type: "changeVideo", data: msg.data }));
        }
      });
    }
  );

  node.createSubscription(
    "std_msgs/msg/String",
    "/hri/display/map",
    (msg: { data: string }) => {
      wss.clients.forEach((client: any) => {
        if (client.readyState === 1) {
          try {
            // Parse the string data as JSON before sending
            const mapData = JSON.parse(msg.data);
            client.send(JSON.stringify({ type: "map", data: mapData }));
          } catch (error) {
            console.error("Failed to parse map data:", error);
          }
        }
      });
    }
  );
  node.createSubscription(
    "std_msgs/msg/String",
    "/hri/display/frida_questions",
    (msg: { data: string }) => {
      wss.clients.forEach((client: any) => {
        if (client.readyState === 1) {
          client.send(JSON.stringify({ type: "question", data: msg.data }));
        }
      });
    }
  );
  node.createSubscription(
    "std_msgs/msg/String",
    "/hri/display/answers",
    (msg: { data: string }) => {
      wss.clients.forEach((client: any) => {
        if (client.readyState === 1) {
          client.send(JSON.stringify({ type: "answer", data: msg.data }));
        }
      });
    }
  );
  publisher = node.createPublisher(
    "std_msgs/msg/Empty",
    "/hri/display/button_press",
    {
      qos: rclnodejs.QoS.profileSystemDefault
    }
  );
  const answerPublisher = node.createPublisher(
    "std_msgs/msg/String",
    "/hri/display/answers",
    {
      qos: rclnodejs.QoS.profileSystemDefault
    }
  );
  interface AnswerMessage {
    type: "answer";
    answer: string;
  }

  wss.on('connection', (ws) => {
    ws.on('message', (message: string ) => {
      try {
        const data: AnswerMessage = JSON.parse(message.toString());
        if (data.type === "answer") {
          // Publish the answer to the ROS topic
          answerPublisher.publish({ data: data.answer });
        }
      } catch (error: unknown) {
        console.error("Error processing message:", error);
      }
    });
  });

  // Gracefully handle SIGINT (Ctrl+C)
  process.on("SIGINT", () => {
    console.log("SIGINT received: Closing node and WebSocket server...");
    node.destroy(); // Cleanup the node
    wss.close(); // Close the WebSocket server
    process.exit(0); // Exit the process
  });

  node.spin();
});
