"use client";

import { MessageCircle } from "lucide-react";
import { AudioStateIndicator } from "../../components/InteractionIndicator";
import { ConnectionStatus } from "../../components/ConnectionStatus";
import { VideoFeed } from "../../components/VideoFeed";
import { StartButton } from "../../components/StartButton";

export default function LaundryPage() {
    return (
        <div className="flex flex-col h-screen bg-(--bg-dark) text-(--text-light) overflow-hidden">
            {/* HEADER */}
            <div className="p-4 border-b border-(--border-light) flex items-center justify-between">
                <h1 className="text-xl font-bold flex items-center">
                    <MessageCircle className="mr-2 h-5 w-5" />
                    Laundry Task
                </h1>

                <div className="flex items-center gap-3">
                    <AudioStateIndicator />
                    <ConnectionStatus />
                </div>
            </div>

            {/* CENTRADO */}
            <div className="flex flex-1 flex-col items-center justify-center p-6 text-center">
                <div className="mb-6 w-full max-w-lg">
                    <StartButton />
                </div>

                {/* VIDEO FEED */}
                <div className="w-full max-w-4xl border-2 border-(--border-light) rounded-lg overflow-hidden shadow-2xl">
                    <VideoFeed />
                </div>
            </div>
        </div>
    );
}
