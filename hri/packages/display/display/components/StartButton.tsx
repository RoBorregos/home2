"use client";

import { useEffect, useState } from "react";
import { Topic } from "roslib";
import { rosClient } from "../RosClient";

export function StartButton() {
  const [publisher, setPublisher] = useState<Topic<{}> | null>(null);

  useEffect(() => {
    const buttonPublisher = new Topic<{}>({
      ros: rosClient,
      name: "/hri/display/button_press",
      messageType: "std_msgs/Empty",
    });
    setPublisher(buttonPublisher);
  }, []);

  const handleClick = () => {
    if (publisher) {
      publisher.publish({});
    }
  };

  return (
    <button
      className="mb-4 px-4 py-2 bg-(--blue) text-white rounded-lg hover:bg-(--blue-hover) transition-colors w-full h-16 text-lg font-semibold"
      onClick={handleClick}
    >
      Start
    </button>
  );
}
