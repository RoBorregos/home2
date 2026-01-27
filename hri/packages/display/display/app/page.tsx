"use client";

import { MessageCircle } from "lucide-react";
import { AudioStateIndicator } from "../components/InteractionIndicator";
import { ConnectionStatus } from "../components/ConnectionStatus";
import { MessagesList } from "../components/MessagesList";
import { MapModal } from "../components/MapModal";
import { QuestionModal } from "../components/QuestionModal";
import { VideoFeed } from "../components/VideoFeed";
import { StartButton } from "../components/StartButton";

export default function Home() {
  return (
    <div className="flex flex-col h-screen bg-(--bg-dark) text-(--text-light) overflow-y-hidden">
      <div className="p-4 border-b border-(--border-light) flex items-center justify-between">
        <h1 className="text-xl font-bold flex items-center">
          <MessageCircle className="mr-2 h-5 w-5" />
          ROS2 Messages
        </h1>
        <div className="flex items-center gap-3">
          <AudioStateIndicator />
          <ConnectionStatus />
        </div>
      </div>

      <div className="grid grid-cols-2 h-full overflow-y-hidden">
        {/* Left column - Messages */}
        <MessagesList />

        {/* Right column - Video and Controls */}
        <div className="sticky top-0 self-start h-[inherit] border-l border-(--border-light) bg-(--bg-dark)">
          <div className="h-full flex flex-col items-center justify-center p-4">
            <StartButton />
            <VideoFeed />
          </div>
        </div>
      </div>

      {/* Modals */}
      <MapModal />
      <QuestionModal />
    </div>
  );
}
