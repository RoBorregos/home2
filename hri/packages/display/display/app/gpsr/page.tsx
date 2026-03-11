"use client";

import { MessageCircle } from "lucide-react";
import { AudioStateIndicator } from "../../components/InteractionIndicator";
import { ConnectionStatus } from "../../components/ConnectionStatus";
import { MessagesList } from "../../components/MessagesList";
import { MapModal } from "../../components/MapModal";
import { QuestionModal } from "../../components/QuestionModal";
import { VideoFeed } from "../../components/VideoFeed";
import { StartButton } from "../../components/StartButton";

export default function GPSRPage() {
    return (
        <div className="flex flex-col h-dvh bg-(--bg-dark) text-(--text-light) overflow-hidden">
            {/* HEADER */}
            <div className="px-[2vw] py-[1.5vh] border-b border-(--border-light) flex items-center justify-between shrink-0">
                <h1 className="text-[clamp(0.875rem,2vw,1.25rem)] font-bold flex items-center">
                    <MessageCircle className="mr-2 h-[1em] w-[1em]" />
                    GPSR Commands
                </h1>
                <div className="flex items-center gap-[clamp(0.5rem,1vw,0.75rem)]">
                    <AudioStateIndicator />
                    <ConnectionStatus />
                </div>
            </div>

            {/* GRID — 1/3 messages, 2/3 video */}
            <div className="grid grid-cols-3 flex-1 min-h-0 overflow-hidden">
                {/* Left column - Messages */}
                <div className="col-span-1 border-r border-(--border-light) overflow-y-auto">
                    <MessagesList />
                </div>

                {/* Center & Right column - Video + Start */}
                <div className="col-span-2 h-full bg-(--bg-dark) overflow-hidden">
                    <div className="h-full flex flex-col items-center justify-center p-[2vmin]">
                        <div className="w-3/4 mb-[2vh]">
                            <StartButton />
                        </div>
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
