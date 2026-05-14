"use client";

import { Message } from "../types";

interface CurrentMessageOverlayProps {
  message: Message;
}

export function CurrentMessageOverlay({ message }: CurrentMessageOverlayProps) {
  return (
    <div className="fixed inset-0 z-100 flex items-center justify-center bg-black/10 backdrop-blur-[1px] pointer-events-none translate-y-32 md:translate-y-48 transition-transform duration-300">
      <div className="bg-(--bg-dark) border-2 border-(--blue) rounded-2xl p-10 max-w-4xl mx-4 shadow-2xl transition-all duration-300 relative">
        <p className="text-4xl text-(--text-light) text-center font-semibold leading-relaxed">
          {message.content}
        </p>
        <p className="text-xl text-(--text-gray) text-center mt-4 opacity-60">
          {message.timestamp.toLocaleTimeString()}
        </p>
      </div>
    </div>
  );
}
