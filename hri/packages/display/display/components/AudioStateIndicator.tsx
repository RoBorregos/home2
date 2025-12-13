"use client";

import { useEffect, useState } from "react";
import { Mic, Speaker, VolumeX } from "lucide-react";
import { Topic } from "roslib";
import { rosClient } from "../RosClient";
import { AudioState } from "../types";

export function AudioStateIndicator() {
  const [audioState, setAudioState] = useState<AudioState>({
    state: "idle",
    vadLevel: 0,
  });

  useEffect(() => {
    // Subscribe to audio state topic
    const audioStateTopic = new Topic<{ data: string }>({
      ros: rosClient,
      name: "/AudioState",
      messageType: "std_msgs/String",
    });

    audioStateTopic.subscribe((msg: { data: string }) => {
      setAudioState((prev) => ({
        ...prev,
        state: msg.data as "idle" | "listening" | "saying",
      }));
    });

    // Subscribe to VAD topic
    const vadTopic = new Topic<{ data: number }>({
      ros: rosClient,
      name: "/hri/speech/vad",
      messageType: "std_msgs/Float32",
    });

    vadTopic.subscribe((msg: { data: number }) => {
      setAudioState((prev) => ({
        ...prev,
        vadLevel: msg.data,
      }));
    });

    return () => {
      audioStateTopic.unsubscribe();
      vadTopic.unsubscribe();
    };
  }, []);

  // Ensure non-idle states auto-timeout back to idle after 10 seconds
  useEffect(() => {
    if (audioState.state !== "idle") {
      const timeout = setTimeout(() => {
        setAudioState((prev) => ({ ...prev, state: "idle" }));
      }, 10_000);
      return () => clearTimeout(timeout);
    }
  }, [audioState.state]);

  const { state, vadLevel } = audioState;

  if (state === "idle") {
    return (
      <div className="flex items-center gap-2 px-3 py-1.5 rounded-full">
        <VolumeX className="h-4 w-4 text-(--text-gray)" />
        <span className="text-xs font-medium text-(--text-gray)">Idle</span>
      </div>
    );
  }

  if (state === "saying") {
    return (
      <div className="flex items-center gap-2 px-3 py-1.5 rounded-full bg-(--purple-bg)">
        <Speaker className="h-4 w-4 text-(--purple) animate-pulse" />
        <span className="text-xs font-medium text-(--purple)">Speaking</span>
      </div>
    );
  }

  // For listening state, show the mic centered
  return (
    <div className="fixed inset-0 z-50 flex items-start justify-center bg-black/10 pointer-events-none pt-35">
      <div className="relative">
        <div className="absolute inset-0 rounded-full bg-(--pulse) opacity-10 animate-[pulse_3s_infinite] scale-110" />
        <div className="absolute inset-0 rounded-full bg-(--pulse) opacity-15 animate-[pulse_3s_infinite_1s] scale-125" />
        <div className="absolute inset-0 rounded-full bg-(--pulse) opacity-20 animate-[pulse_3s_infinite_2s] scale-150" />

        <div className="relative z-10 h-70 w-70 rounded-full bg-(--pulse) shadow-lg flex items-center justify-center">
          <Mic className="h-50 w-50 text-white/90 drop-shadow-md" />
        </div>

        <div
          className="absolute inset-0 rounded-full border-4 border-(--pulse) opacity-0 transition-all duration-300"
          style={{
            transform: `scale(${1 + (vadLevel || 0)})`,
            opacity: (vadLevel || 0) * 0.8,
          }}
        />
      </div>
    </div>
  );
}
