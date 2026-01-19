"use client";

import { MessageCircle } from "lucide-react";
import { AudioStateIndicator } from "../../components/AudioStateIndicator";
import { ConnectionStatus } from "../../components/ConnectionStatus";
import { MessagesList } from "../../components/MessagesList";
import { MapModal } from "../../components/MapModal";
import { QuestionModal } from "../../components/QuestionModal";
import { VideoFeed } from "../../components/VideoFeed";
import { StartButton } from "../../components/StartButton";

export default function GPSRPage() {
    return (
        <div className="flex flex-col h-screen bg-(--bg-dark) text-(--text-light) overflow-y-hidden">
            {/* HEADER */}
            <div className="p-4 border-b border-(--border-light) flex items-center justify-between">
                <h1 className="text-xl font-bold flex items-center">
                    <MessageCircle className="mr-2 h-5 w-5" />
                    GPSR Commands
                </h1>
                <div className="flex items-center gap-3">
                    <AudioStateIndicator />
                    <ConnectionStatus />
                </div>
            </div>

            {/* GRID */}
            <div className="grid grid-cols-3 h-full overflow-y-hidden">
                {/* Left column - Messages */}
                <div className="col-span-1 border-r border-(--border-light) overflow-y-auto">
                    <MessagesList />
                </div>

                {/* Center & Right column - Video + Start */}
                <div className="col-span-2 sticky top-0 self-start h-[inherit] bg-(--bg-dark)">
                    <div className="h-full flex flex-col items-center justify-center p-4">
                        <div className="w-3/4 mb-4">
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
