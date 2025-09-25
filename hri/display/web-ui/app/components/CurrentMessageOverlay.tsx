"use client";

import { Mic } from "lucide-react";
import { Message } from "./types";

interface CurrentMessageOverlayProps {
  message: Message;
}

export function CurrentMessageOverlay({ message }: CurrentMessageOverlayProps) {
  return (
    <div
      key={message.timestamp.getTime()}
      className="fixed bottom-32 inset-x-0 mx-auto bg-gray-500/20 text-white py-6 rounded-lg shadow-lg z-50 animate-fadeIn w-fit max-w-[90vw]"
    >
      <div className="flex items-center justify-center gap-4">
        <Mic className="h-20 w-20 animate-pulse text-white" />
        <p className="text-7xl font-medium tracking-wide">{message.content}</p>
      </div>
    </div>
  );
}
