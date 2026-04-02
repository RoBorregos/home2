"use client";

import { Message } from "../types";

interface CurrentMessageOverlayProps {
  message: Message;
}

export function CurrentMessageOverlay({ message }: CurrentMessageOverlayProps) {
  return (
    <div className="fixed inset-0 z-[100] flex items-center justify-center bg-black/10 backdrop-blur-[1px] pointer-events-none translate-y-48 md:translate-y-64 transition-transform duration-300">
      <div className="bg-(--bg-dark) border-2 border-(--blue) rounded-2xl p-4 md:p-8 max-w-sm md:max-w-2xl mx-4 shadow-2xl transition-all duration-300 relative">
        <p className="text-xl md:text-3xl text-(--text-light) text-center font-semibold leading-relaxed">
          {message.content}
        </p>
        <p className="text-[10px] md:text-xs text-(--text-gray) text-center mt-3 opacity-60">
          {message.timestamp.toLocaleTimeString()}
        </p>
      </div>
    </div>
  );
}
