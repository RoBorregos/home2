"use client";

import { MessageCircle } from "lucide-react";
import { AudioStateIndicator } from "../../components/InteractionIndicator";
import { ConnectionStatus } from "../../components/ConnectionStatus";
import { VideoFeed } from "../../components/VideoFeed";
import { StartButton } from "../../components/StartButton";

export default function StoreGroceriesPage() {
    return (
        <div className="flex flex-col h-dvh bg-(--bg-dark) text-(--text-light) overflow-hidden">
            {/* HEADER */}
            <div className="px-[2vw] py-[1.5vh] border-b border-(--border-light) flex items-center justify-between shrink-0">
                <h1 className="text-[clamp(0.875rem,2vw,1.25rem)] font-bold flex items-center">
                    <MessageCircle className="mr-2 h-[1em] w-[1em]" />
                    Storing Groceries
                </h1>

                <div className="flex items-center gap-[clamp(0.5rem,1vw,0.75rem)]">
                    <AudioStateIndicator />
                    <ConnectionStatus />
                </div>
            </div>

            {/* CONTENIDO CENTRADO */}
            <div className="flex flex-1 flex-col items-center justify-center p-[3vmin] text-center min-h-0">
                <div className="mb-[2vh]">
                    <StartButton />
                </div>

                {/* VIDEO CENTRADO */}
                <div className="w-full max-w-[80vw]">
                    <VideoFeed />
                </div>
            </div>

            {/* FOOTER */}
            <div className="py-[1vh] border-t border-(--border-light) bg-(--bg-darker) text-center shrink-0">
            </div>
        </div>
    );
}
