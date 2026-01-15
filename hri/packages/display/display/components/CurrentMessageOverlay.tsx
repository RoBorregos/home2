"use client";

import { Message } from "../types";

interface CurrentMessageOverlayProps {
  message: Message;
}

export function CurrentMessageOverlay({ message }: CurrentMessageOverlayProps) {
  return (
    <div className="fixed inset-0 z-40 flex items-center justify-center bg-black/50 backdrop-blur-sm pointer-events-none">
      <div className="bg-(--bg-dark) border-2 border-(--blue) rounded-2xl p-8 max-w-2xl mx-4 shadow-2xl">
        <p className="text-3xl text-(--text-light) text-center font-medium">
          {message.content}
        </p>
        <p className="text-sm text-(--text-gray) text-center mt-4">
          {message.timestamp.toLocaleTimeString()}
        </p>
      </div>
    </div>
  );
}
