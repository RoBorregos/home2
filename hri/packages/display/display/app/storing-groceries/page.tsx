"use client";

import { MessageCircle } from "lucide-react";
import { AudioStateIndicator } from "../../components/AudioStateIndicator";
import { ConnectionStatus } from "../../components/ConnectionStatus";
import { VideoFeed } from "../../components/VideoFeed";
import { StartButton } from "../../components/StartButton";

export default function StoreGroceriesPage() {
    return (
        <div className="flex flex-col h-screen bg-(--bg-dark) text-(--text-light) overflow-hidden">
            {/* HEADER */}
            <div className="p-4 border-b border-(--border-light) flex items-center justify-between">
                <h1 className="text-xl font-bold flex items-center">
                    <MessageCircle className="mr-2 h-5 w-5" />
                    Storing Groceries
                </h1>

                <div className="flex items-center gap-3">
                    <AudioStateIndicator />
                    <ConnectionStatus />
                </div>
            </div>

            {/* CONTENIDO CENTRADO */}
            <div className="flex flex-1 flex-col items-center justify-center p-6 text-center">
                <div className="mb-6">
                    <StartButton />
                </div>

                {/* VIDEO CENTRADO */}
                <div className="w-full max-w-3xl">
                    <VideoFeed />
                </div>
            </div>

            {/* FOOTER */}
            <div className="p-3 border-t border-(--border-light) bg-(--bg-darker) text-center">
                <p className="text-sm text-(--text-gray)">Video Display Ready</p>
            </div>
        </div>
    );
}
