"use client";

import { useEffect, useState } from "react";
import dynamic from "next/dynamic";
import { Topic } from "roslib";
import { rosClient } from "../RosClient";
import { setCurrentVideoTopic } from "./Captures";

const MjpegStream = dynamic(() => import("./video"), { ssr: false });

interface VideoFeedProps {
  defaultTopic?: string;
}

export function VideoFeed({
  defaultTopic = "/vision/camera/image_oriented",
}: VideoFeedProps) {
  const [videoTopic, setVideoTopic] = useState<string>(defaultTopic);

  useEffect(() => {
    setCurrentVideoTopic(videoTopic);
  }, [videoTopic]);

  useEffect(() => {
    const changeVideoTopic = new Topic<{ data: string }>({
      ros: rosClient,
      name: "/hri/display/change_video",
      messageType: "std_msgs/String",
    });

    changeVideoTopic.subscribe((msg: { data: string }) => {
      setVideoTopic(msg.data);
    });

    return () => {
      changeVideoTopic.unsubscribe();
    };
  }, []);

  return (
    <div className="w-full h-full">
      <MjpegStream
        streamUrl={`http://localhost:8080/stream?topic=${videoTopic}`}
      />
    </div>
  );
}
