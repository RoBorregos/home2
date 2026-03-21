"use client";

import { Message } from "../types";

interface CurrentMessageOverlayProps {
  message: Message;
}

export function CurrentMessageOverlay({ message }: CurrentMessageOverlayProps) {
  return (
    <div className="fixed inset-0 z-60 flex items-center justify-center bg-black/20 backdrop-blur-[2px] pointer-events-none translate-y-32 md:translate-y-48 transition-transform duration-300">
      <div className="bg-(--bg-dark) border-2 border-(--blue) rounded-2xl p-4 md:p-8 max-w-sm md:max-w-2xl mx-4 shadow-2xl transition-all duration-300">
        <p className="text-xl md:text-3xl text-(--text-light) text-center font-medium">
          {message.content}
        </p>
        <p className="text-xs md:text-sm text-(--text-gray) text-center mt-2 md:mt-4">
          {message.timestamp.toLocaleTimeString()}
        </p>
      </div>
    </div>
  );
}
