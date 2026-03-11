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
    <div className="flex flex-col h-dvh bg-(--bg-dark) text-(--text-light) overflow-hidden">
      {/* Header */}
      <div className="px-[3vw] md:px-[2vw] py-[1vh] md:py-[1.5vh] border-b border-(--border-light) flex items-center justify-between shrink-0">
        <h1 className="text-[clamp(0.75rem,2.5vw,1.25rem)] md:text-[clamp(0.875rem,2vw,1.25rem)] font-bold flex items-center">
          <MessageCircle className="mr-1.5 md:mr-2 h-[1em] w-[1em]" />
          ROS2 Messages
        </h1>
        <div className="flex items-center gap-[clamp(0.25rem,1.5vw,0.75rem)] md:gap-[clamp(0.5rem,1vw,0.75rem)]">
          <AudioStateIndicator />
          <ConnectionStatus />
        </div>
      </div>

      {/* Main content: stacks on small screens, 2-col grid on md+ */}
      <div className="flex flex-col md:grid md:grid-cols-2 flex-1 min-h-0 overflow-hidden">
        <MessagesList />
        <div className="h-full border-t md:border-t-0 md:border-l border-(--border-light) bg-(--bg-dark) overflow-hidden min-h-[30vh] md:min-h-0">
          <div className="h-full flex flex-col items-center justify-center p-[2vmin]">
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
