"use client";

import { MessageCircle, Mic, Speaker, Star } from "lucide-react";
import { Message } from "../types";

interface MessageItemProps {
  message: Message;
  isLatest: boolean;
}

export function MessageItem({ message, isLatest }: MessageItemProps) {
  return (
    <div
      className={`
        p-3 rounded-lg animate-fadeIn transition-all duration-300
        ${
          message.type === "heard"
            ? "bg-[oklch(0.488_0.243_264.376/20%)] border-l-4 border-l-[oklch(0.488_0.243_264.376)]"
            : message.type === "spoken"
            ? "bg-[oklch(0.627_0.265_303.9/20%)] border-l-4 border-l-[oklch(0.627_0.265_303.9)]"
            : "bg-[oklch(0.9_0.3_60/20%)] border-l-4 border-l-[oklch(0.9_0.3_60)]"
        }
        ${
          isLatest
            ? "ring-2 ring-offset-2 ring-offset-[oklch(0.145_0_0)] ring-[oklch(0.985_0_0/10%)]"
            : ""
        }
      `}
    >
      <div className="flex items-start gap-2">
        {message.type === "spoken" || message.type === "text_spoken" ? (
          <Speaker className="h-5 w-5 text-[oklch(0.627_0.265_303.9)] mt-0.5 flex-shrink-0" />
        ) : message.type === "answer" ? (
          <MessageCircle className="h-5 w-5 text-[oklch(0.4_0.3_120)] mt-0.5 flex-shrink-0" />
        ) : message.type === "heard" ? (
          <Mic className="h-5 w-5 text-[oklch(0.488_0.243_264.376)] mt-0.5 flex-shrink-0" />
        ) : (
          <Star className="h-5 w-5 text-[oklch(0.9_0.3_60)] mt-0.5 flex-shrink-0" />
        )}
        <div className="flex-1 min-w-0">
          <p className="font-medium text-sm text-[oklch(0.708_0_0)]">
            {message.type === "spoken" || message.type === "text_spoken"
              ? "Spoken"
              : message.type === "answer"
              ? "Answer"
              : message.type === "heard"
              ? "Heard"
              : "Keyword"}
          </p>
          <p className="text-lg font-bold break-words">
            {message.content}
          </p>
          <p className="text-xs text-[oklch(0.708_0_0)] mt-1">
            {message.timestamp.toLocaleTimeString([], {
              hour: "2-digit",
              minute: "2-digit",
              second: "2-digit",
            })}
          </p>
        </div>
      </div>
    </div>
  );
}