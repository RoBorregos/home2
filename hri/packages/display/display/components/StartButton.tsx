"use client";

import { useEffect, useState } from "react";
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
        className="fixed bottom-2 right-2 w-3 h-3 bg-white/5 hover:bg-white/20 rounded-full transition-all duration-300 z-50 cursor-default"
        onClick={() => setIsMaximized(true)}
        aria-label="Setup attempt"
      />
    );
  }

  return (
    <button
      className="mb-4 px-4 bg-(--blue) text-white rounded-lg hover:bg-(--blue-hover) transition-all duration-300 font-semibold flex items-center justify-center w-full h-16"
      onClick={handleClick}
    >
    </button>
  );
}
