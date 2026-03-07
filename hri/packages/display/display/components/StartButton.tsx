"use client";

import { useEffect, useState } from "react";
import { Play } from "lucide-react";
import { Topic } from "roslib";
import { rosClient } from "../RosClient";

export function StartButton() {
  const [publisher, setPublisher] = useState<Topic<{}> | null>(null);
  const [isTaskActive, setIsTaskActive] = useState(false);
  const [isMaximized, setIsMaximized] = useState(false);

  useEffect(() => {
    const buttonPublisher = new Topic<{}>({
      ros: rosClient,
      name: "/hri/display/button_press",
      messageType: "std_msgs/Empty",
    });
    setPublisher(buttonPublisher);

    const taskStatusTopic = new Topic<{ data: string }>({
      ros: rosClient,
      name: "/hri/display/task_status",
      messageType: "std_msgs/String",
    });

    taskStatusTopic.subscribe((msg: { data: string }) => {
      const active = msg.data === "active";
      setIsTaskActive(active);
      if (active) {
        setIsMaximized(false);
      }
    });

    return () => {
      taskStatusTopic.unsubscribe();
    };
  }, []);

  const handleClick = () => {
    if (publisher) {
      publisher.publish({});
      setIsMaximized(false);
    }
  };

  if (isTaskActive && !isMaximized) {
    return (
      <button
        className="fixed bottom-4 right-4 w-10 h-10 bg-(--blue)/80 hover:bg-(--blue) text-white rounded-full transition-all duration-300 z-50 flex items-center justify-center shadow-lg"
        onClick={() => setIsMaximized(true)}
        aria-label="Show start button"
      >
        <Play className="h-5 w-5" />
      </button>
    );
  }

  return (
    <button
      className="mb-4 px-4 bg-(--blue) text-white rounded-lg hover:bg-(--blue-hover) transition-all duration-300 font-semibold flex items-center justify-center gap-2 w-full h-16 text-lg"
      onClick={handleClick}
    >
      <Play className="h-5 w-5" />
      Start
    </button>
  );
}
