"use client";

import { Mic, Speaker, VolumeX } from "lucide-react";
import { AudioStateIndicatorProps } from "./types";

export function AudioStateIndicator({ state }: AudioStateIndicatorProps) {
  const { state: audioState, vadLevel } = state;

  if (audioState === "idle") {
    return (
      <div className="flex items-center gap-2 px-3 py-1.5 rounded-full bg-[oklch(0.3_0_0/30%)]">
        <VolumeX className="h-4 w-4 text-[oklch(0.708_0_0)]" />
        <span className="text-xs font-medium text-[oklch(0.708_0_0)]">
          Idle
        </span>
      </div>
    );
  }

  if (audioState === "saying") {
    return (
      <div className="flex items-center gap-2 px-3 py-1.5 rounded-full bg-[oklch(0.627_0.265_303.9/20%)]">
        <Speaker className="h-4 w-4 text-[oklch(0.627_0.265_303.9)] animate-pulse" />
        <span className="text-xs font-medium text-[oklch(0.627_0.265_303.9)]">
          Speaking
        </span>
      </div>
    );
  }

  // For listening state, show the mic centered
  return (
    <div className="fixed inset-0 z-50 flex items-start justify-center bg-black/10 pointer-events-none pt-35 ">
      <div className="relative">
        <div className="absolute inset-0 rounded-full bg-[oklch(0.5_0.25_260)] opacity-10 animate-[pulse_3s_infinite] scale-110" />
        <div className="absolute inset-0 rounded-full bg-[oklch(0.5_0.25_260)] opacity-15 animate-[pulse_3s_infinite_1s] scale-125" />
        <div className="absolute inset-0 rounded-full bg-[oklch(0.5_0.25_260)] opacity-20 animate-[pulse_3s_infinite_2s] scale-150" />

        <div className="relative z-10 h-70 w-70 rounded-full bg-[oklch(0.5_0.25_260)] shadow-lg flex items-center justify-center">
          <Mic className="h-50 w-50 text-white/90 drop-shadow-md" />
        </div>

        <div 
          className="absolute inset-0 rounded-full border-4 border-[oklch(0.5_0.25_260)] opacity-0 transition-all duration-300"
          style={{
            transform: `scale(${1 + (vadLevel || 0)})`,
            opacity: (vadLevel || 0) * 0.8
          }}
        />
        
      </div>
    </div>   
  );
}
