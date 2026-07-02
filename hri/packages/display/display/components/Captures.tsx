"use client";

import { useEffect, useState } from "react";
import { Camera, X } from "lucide-react";
import { Topic } from "roslib";
import { rosClient } from "../RosClient";

// Task name used to pick the run folder server-side; set by useCaptures.
let logTask = "default";

export function setLogTask(task: string) {
  logTask = task;
}

// Fire-and-forget append to this run's messages.log (via /api/log).
export function logEvent(kind: string, text: string) {
  fetch("/api/log", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ task: logTask, kind, text }),
  }).catch(() => {});
}

// Last topic shown by VideoFeed — fallback when a capture has no topic.
let currentVideoTopic = "";

export function setCurrentVideoTopic(topic: string) {
  currentVideoTopic = topic;
}

export interface Capture {
  label: string;
  time: string;
  dataUrl: string | null;
}

/**
 * Subscribes to /hri/display/capture (plain-text label, or JSON
 * {"label", "topic"}). Each request snapshots the topic via the
 * /api/capture route (web_video_server), which also saves it to
 * logs/captures/<task>_<boot>/ on disk.
 */
export function useCaptures(task: string, defaultTopic = ""): Capture[] {
  const [captures, setCaptures] = useState<Capture[]>([]);

  useEffect(() => {
    setLogTask(task);
    const captureTopic = new Topic<{ data: string }>({
      ros: rosClient,
      name: "/hri/display/capture",
      messageType: "std_msgs/String",
    });

    captureTopic.subscribe(async (msg: { data: string }) => {
      let label = msg.data;
      let topic = "";
      try {
        const data = JSON.parse(msg.data);
        label = data.label || "capture";
        topic = data.topic || "";
      } catch {
        // plain-text label
      }
      if (!topic) topic = currentVideoTopic || defaultTopic;

      const time = new Date().toLocaleTimeString();
      let dataUrl: string | null = null;
      try {
        const res = await fetch("/api/capture", {
          method: "POST",
          headers: { "Content-Type": "application/json" },
          body: JSON.stringify({ label, topic, task }),
        });
        dataUrl = (await res.json()).dataUrl ?? null;
      } catch {
        // keep the entry so the gallery shows the miss
      }
      setCaptures((prev) => [...prev, { label, time, dataUrl }]);
    });

    return () => {
      captureTopic.unsubscribe();
    };
  }, [task, defaultTopic]);

  return captures;
}

export function CapturesGallery({ captures }: { captures: Capture[] }) {
  if (captures.length === 0) {
    return (
      <div className="h-full flex items-center justify-center text-(--text-gray)">
        <p className="flex items-center gap-2">
          <Camera className="h-5 w-5" /> No captures yet
        </p>
      </div>
    );
  }
  return (
    <div className="h-full overflow-y-auto p-4">
      <div className="grid grid-cols-2 xl:grid-cols-3 gap-4">
        {captures.map((c, i) => (
          <div
            key={i}
            className="rounded-lg border border-(--border-light) bg-(--bg-darker) overflow-hidden"
          >
            {c.dataUrl ? (
              <img src={c.dataUrl} alt={c.label} className="w-full" />
            ) : (
              <div className="aspect-video flex items-center justify-center text-(--text-gray) text-sm">
                no frame
              </div>
            )}
            <p className="px-2 py-1.5 text-xs text-(--text-gray) text-center">
              {c.label} — {c.time}
            </p>
          </div>
        ))}
      </div>
    </div>
  );
}

/** Header chip counting captures; click opens the gallery as an overlay. */
export function CapturesButton({ captures }: { captures: Capture[] }) {
  const [open, setOpen] = useState(false);

  return (
    <>
      <button
        onClick={() => setOpen(true)}
        className="flex items-center gap-1.5 px-3 py-1 rounded-full text-xs font-semibold bg-amber-500/15 text-amber-400 ring-1 ring-amber-500/30 whitespace-nowrap"
      >
        <Camera className="h-4 w-4" />
        <span>{captures.length}</span>
      </button>
      {open && (
        <div className="fixed inset-0 z-50 bg-black/80 flex flex-col">
          <div className="p-3 border-b border-(--border-light) flex items-center justify-between shrink-0 bg-(--bg-dark)">
            <h2 className="text-lg font-bold flex items-center gap-2 text-(--text-light)">
              <Camera className="h-5 w-5" /> Captures
            </h2>
            <button
              onClick={() => setOpen(false)}
              className="p-1.5 rounded-lg text-(--text-gray) hover:text-(--text-light)"
            >
              <X className="h-5 w-5" />
            </button>
          </div>
          <div className="flex-1 min-h-0 bg-(--bg-dark)">
            <CapturesGallery captures={captures} />
          </div>
        </div>
      )}
    </>
  );
}
