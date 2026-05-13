"use client";

import { Volume2, Mic, Tag, MessageSquare } from "lucide-react";
import { Message } from "../types";

interface MessageItemProps {
  message: Message;
  isLatest: boolean;
}

export function MessageItem({ message, isLatest }: MessageItemProps) {
  const getIcon = () => {
    switch (message.type) {
      case "heard":
        return <Mic className="h-8 w-8" />;
      case "spoken":
      case "text_spoken":
        return <Volume2 className="h-8 w-8" />;
      case "keyword":
        return <Tag className="h-8 w-8" />;
      case "answer":
      case "user_message":
        return <MessageSquare className="h-8 w-8" />;
      default:
        return <MessageSquare className="h-4 w-4" />;
    }
  };

  const getTypeColor = () => {
    switch (message.type) {
      case "heard":
        return "text-[oklch(0.488_0.243_264.376)]";
      case "spoken":
      case "text_spoken":
        return "text-[oklch(0.627_0.265_303.9)]";
      case "keyword":
        return "text-[oklch(0.704_0.191_22.216)]";
      case "answer":
      case "user_message":
        return "text-[oklch(0.6_0.2_140)]";
      default:
        return "text-[var(--text-gray)]";
    }
  };

  return (
    <div
      className={`p-6 rounded-xl border-2 transition-all ${
        isLatest
          ? "bg-(--bg-darker) border-(--border-light) shadow-lg"
          : "bg-(--bg-dark) border-(--border-light)"
      }`}
    >
      <div className="flex items-start gap-5">
        <div className={`mt-1 ${getTypeColor()}`}>{getIcon()}</div>
        <div className="flex-1 min-w-0">
          <div className="flex items-center gap-3 mb-2">
            <span className={`text-lg font-bold ${getTypeColor()}`}>
              {message.type.toUpperCase()}
            </span>
            <span className="text-base text-(--text-gray)">
              {message.timestamp.toLocaleTimeString()}
            </span>
          </div>
          <p className="text-[1.4rem] text-(--text-light) wrap-break-word leading-relaxed">
            {message.content}
          </p>
        </div>
      </div>
    </div>
  );
}
