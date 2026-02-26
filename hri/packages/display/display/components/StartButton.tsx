"use client";

import { useEffect, useState } from "react";
import { Topic } from "roslib";
import { rosClient } from "../RosClient";

export function StartButton() {
  const [publisher, setPublisher] = useState<Topic<{}> | null>(null);
  const [isTaskActive, setIsTaskActive] = useState(false);

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
      setIsTaskActive(msg.data === "active");
    });

    return () => {
      taskStatusTopic.unsubscribe();
    };
  }, []);

  const handleClick = () => {
    if (publisher) {
      publisher.publish({});
    }
  };

  return (
    <button
      className={`mb-4 px-4 bg-(--blue) text-white rounded-lg hover:bg-(--blue-hover) transition-all duration-300 font-semibold flex items-center justify-center ${isTaskActive ? "w-32 h-10 text-sm" : "w-full h-16 text-lg"
        }`}
      onClick={handleClick}
    >
      Start
    </button>
  );
}
