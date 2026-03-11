"use client";

import { useEffect, useState } from "react";
import dynamic from "next/dynamic";
import { Topic } from "roslib";
import { rosClient } from "../RosClient";

const MjpegStream = dynamic(() => import("./video"), { ssr: false });

export function VideoFeed() {
  const [videoTopic, setVideoTopic] = useState<string>(
    "/zed/zed_node/rgb/image_rect_color"
  );

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
    <div className="h-full flex flex-col items-center justify-center p-[1vmin]">
      <p className="text-[clamp(0.625rem,1.5vw,1.25rem)] mb-[0.5vh] md:mb-[1vh] text-(--text-light)">
        Video feed at {videoTopic}
      </p>
      <MjpegStream
        streamUrl={`http://localhost:8080/stream?topic=${videoTopic}`}
      />
    </div>
  );
}
