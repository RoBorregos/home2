import React, { useEffect, useState } from "react";

const App: React.FC = () => {
  const [messages, setMessages] = useState<string[]>([]);

  useEffect(() => {
    const socket = new WebSocket("ws://localhost:8000");

    socket.onmessage = (event) => {
      const data = JSON.parse(event.data);
      if (data.type === "heard") {
        setMessages((prev) => [...prev, `Heard: ${data.data}`]);
      } else if (data.type === "spoken") {
        setMessages((prev) => [...prev, `Spoken: ${data.data}`]);
      }
    };

    socket.onclose = () => console.log("WebSocket disconnected");
    socket.onerror = (error) => console.error("WebSocket error:", error);

    return () => {
      socket.close();
    };
  }, []);

  return (
    <div className="p-4 text-black">
      <h1 className="text-2xl font-bold mb-4">ROS2 Messages</h1>
      <ul className="list-disc pl-5">
        {messages.map((msg, index) => (
          <li key={index} className="mb-2">
            {msg}
          </li>
        ))}
      </ul>
    </div>
  );
};

export default App;
