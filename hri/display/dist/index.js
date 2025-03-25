"use strict";
var __importDefault = (this && this.__importDefault) || function (mod) {
    return (mod && mod.__esModule) ? mod : { "default": mod };
};
Object.defineProperty(exports, "__esModule", { value: true });
const dotenv_1 = __importDefault(require("dotenv"));
const express_1 = __importDefault(require("express"));
const ws_1 = require("ws");
const rclnodejs_1 = __importDefault(require("rclnodejs"));
const cors_1 = __importDefault(require("cors"));
dotenv_1.default.config();
const app = (0, express_1.default)();
app.use(express_1.default.json());
app.use((0, cors_1.default)());
app.get("/", (req, res) => {
    res.send("Hello World From the Typescript Server!");
});
const port = process.env.PORT || 8000;
const server = app.listen(port, () => {
    console.log(`Example app listening on port ${port}`);
});
const wss = new ws_1.WebSocketServer({ server });
rclnodejs_1.default.init().then(() => {
    const node = new rclnodejs_1.default.Node("publisher_example_node");
    node.createSubscription("std_msgs/msg/String", "/speech/raw_command", (msg) => {
        console.log(`I heard: [${msg.data}]`);
        wss.clients.forEach((client) => {
            if (client.readyState === 1) {
                client.send(JSON.stringify({ type: "heard", data: msg.data }));
            }
        });
    });
    node.createSubscription("std_msgs/msg/String", "/speech/text_spoken", (msg) => {
        console.log(`I said: [${msg.data}]`);
        wss.clients.forEach((client) => {
            if (client.readyState === 1) {
                client.send(JSON.stringify({ type: "spoken", data: msg.data }));
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
