"use client";

import { useEffect, useState } from "react";
import { Flame } from "lucide-react";
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

  const isShrunk = isTaskActive && !isMaximized;

  return (
    <div className="w-full h-16 mb-4 relative">
      <button
        className={`
          relative w-full h-full bg-(--blue) hover:bg-(--blue-hover) text-white rounded-lg font-semibold text-lg flex items-center justify-center gap-2 shadow-lg z-10
          transition-all duration-200 ease-out
          ${isShrunk ? "opacity-0 scale-95 pointer-events-none" : "opacity-100 scale-100 pointer-events-auto"}
        `}
        onClick={handleClick}
        aria-label="Start task"
      >
        <Flame className="h-5 w-5" />
        <span>Start</span>
      </button>

      <button
        className={`
          fixed bottom-4 right-4 w-20 h-20 bg-(--blue)/80 hover:bg-(--blue) text-white rounded-full flex items-center justify-center shadow-lg z-50
          transition-all duration-200 ease-out
          ${isShrunk ? "opacity-100 scale-100 pointer-events-auto" : "opacity-0 scale-95 pointer-events-none"}
        `}
        onClick={() => setIsMaximized(true)}
        aria-label="Show start button"
      >
        <Flame className="h-10 w-10" />
      </button>
    </div>
  );
}
