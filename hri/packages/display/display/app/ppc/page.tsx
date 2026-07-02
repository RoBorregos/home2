"use client";

import { useEffect, useState } from "react";
import {
  MessageCircle,
  Flame,
  Play,
  Eye,
  Package,
  Coffee,
  CheckCircle2,
} from "lucide-react";
import { Topic } from "roslib";
import { AudioStateIndicator } from "../../components/InteractionIndicator";
import { ConnectionStatus } from "../../components/ConnectionStatus";
import { MessagesList } from "../../components/MessagesList";
import { MapModal } from "../../components/MapModal";
import { QuestionModal } from "../../components/QuestionModal";
import { VideoFeed } from "../../components/VideoFeed";
import { StartButton } from "../../components/StartButton";
import {
  CapturesButton,
  CapturesGallery,
  logEvent,
  useCaptures,
} from "../../components/Captures";
import { rosClient } from "../../RosClient";

type DisplayMode = "button" | "camera" | "logs" | "both" | "gallery";

interface TaskStep {
  key: string;
  label: string;
  display: DisplayMode;
  icon: React.ReactNode;
}

const TASK_STEPS: TaskStep[] = [
  { key: "wait_for_button", label: "Wait Start", display: "button", icon: <Flame className="h-4 w-4" /> },
  { key: "start", label: "Starting", display: "camera", icon: <Play className="h-4 w-4" /> },
  { key: "perceive_table", label: "Perceive Table", display: "both", icon: <Eye className="h-4 w-4" /> },
  { key: "cleanup_phase", label: "Cleanup Phase", display: "both", icon: <Package className="h-4 w-4" /> },
  { key: "breakfast_phase", label: "Breakfast Phase", display: "both", icon: <Coffee className="h-4 w-4" /> },
  { key: "end", label: "Finished", display: "gallery", icon: <CheckCircle2 className="h-4 w-4" /> },
];

function getStepKey(raw: string): string {
  const s = raw.toLowerCase();
  
  if (s === "wait_for_button") return "wait_for_button";
  if (s === "start") return "start";
  
  if (["perceive_table", "announce_objects", "sort_objects"].includes(s)) return "perceive_table";
  
  if (["scan_cabinet_shelves", "cleanup_loop", "pick_object", "determine_placement", "check_dishwasher", "request_dishwasher_help", "navigate_to_placement", "place_object"].includes(s)) return "cleanup_phase";
  
  if (["start_breakfast_prep", "get_breakfast_items", "navigate_to_item_source", "pick_breakfast_item", "navigate_to_dining", "pour_into_bowl", "place_breakfast_item"].includes(s)) return "breakfast_phase";
  
  if (s === "end" || s === "debug") return "end";

  return "wait_for_button"; // Default
}

function getStepIndex(key: string): number {
  return TASK_STEPS.findIndex((s) => s.key === key);
}

function StepPill({
  step,
  state,
}: {
  step: TaskStep;
  state: "done" | "active" | "pending";
}) {
  const base =
    "flex items-center gap-1.5 px-2.5 py-1 rounded-full text-xs font-medium transition-all duration-300 whitespace-nowrap";
  const styles = {
    done: "bg-emerald-500/20 text-emerald-400",
    active: "bg-(--blue-bg) text-(--blue) ring-1 ring-(--blue)/40 shadow-md shadow-(--blue)/10 animate-pulse",
    pending: "bg-white/5 text-(--text-gray)/60",
  };

  return (
    <div className={`${base} ${styles[state]}`}>
      {step.icon}
      <span className="hidden lg:inline">{step.label}</span>
    </div>
  );
}

// ─── Main page ──
export default function PPCPage() {
  const [currentRawStep, setCurrentRawStep] = useState<string>("wait_for_button");
  const captures = useCaptures("ppc", "/vision/detections_image");

  useEffect(() => {
    const taskTopic = new Topic<{ data: string }>({
      ros: rosClient,
      name: "/pickandplace/display/task_step",
      messageType: "std_msgs/String",
    });

    taskTopic.subscribe((msg: { data: string }) => {
      logEvent("step", msg.data.trim().toLowerCase());
      setCurrentRawStep(msg.data.trim().toLowerCase());
    });

    return () => {
      taskTopic.unsubscribe();
    };
  }, []);

  const currentStep = getStepKey(currentRawStep);
  const activeIndex = getStepIndex(currentStep);
  const activeStep = TASK_STEPS[activeIndex] ?? TASK_STEPS[0];
  const displayMode = activeStep.display;

  return (
    <div className="flex flex-col h-screen bg-(--bg-dark) text-(--text-light) overflow-hidden">
      {/* ── HEADER ──*/}
      <div className="p-3 border-b border-(--border-light) flex items-center justify-between shrink-0">
        <div className="flex items-center gap-3">
          <h1 className="text-lg font-bold flex items-center gap-2">
            <MessageCircle className="h-5 w-5" />
            Pick and Place Challenge
          </h1>
        </div>
        <div className="flex items-center gap-3">
          <CapturesButton captures={captures} />
          <AudioStateIndicator />
          <ConnectionStatus />
        </div>
      </div>

      {/* ── STEP PROGRESS BAR ── */}
      <div className="px-3 py-2 border-b border-(--border-light) shrink-0 overflow-x-auto">
        <div className="flex items-center gap-1.5">
          {TASK_STEPS.map((step, i) => (
            <StepPill
              key={step.key}
              step={step}
              state={
                i < activeIndex
                  ? "done"
                  : i === activeIndex
                    ? "active"
                    : "pending"
              }
            />
          ))}
        </div>
      </div>

      {/* ── MAIN CONTENT ── */}
      <div className="flex-1 min-h-0 overflow-hidden">
        {/* BUTTON mode: centered start button */}
        {displayMode === "button" && (
          <div className="h-full flex items-center justify-center p-8">
            <div className="w-full max-w-lg">
              <StartButton size="xl" />
            </div>
          </div>
        )}

        {/* CAMERA mode: full-width camera feed */}
        {displayMode === "camera" && (
          <div className="h-full flex flex-col items-center justify-center p-4">
            <VideoFeed defaultTopic="/vision/detections_image" />
          </div>
        )}

        {/* LOGS mode: full-width messages */}
        {displayMode === "logs" && (
          <div className="h-full overflow-y-auto">
            <MessagesList />
          </div>
        )}

        {/* BOTH mode: split view */}
        {displayMode === "both" && (
          <div className="grid grid-cols-2 h-full overflow-hidden">
            {/* Left - Messages */}
            <div className="border-r border-(--border-light) overflow-y-auto">
              <MessagesList />
            </div>
            {/* Right - Camera */}
            <div className="flex flex-col items-center justify-center p-4">
              <VideoFeed defaultTopic="/vision/detections_image" />
            </div>
          </div>
        )}

        {/* GALLERY mode (end of task): messages + captured snapshots */}
        {displayMode === "gallery" && (
          <div className="grid grid-cols-2 h-full overflow-hidden">
            <div className="border-r border-(--border-light) overflow-y-auto">
              <MessagesList />
            </div>
            <CapturesGallery captures={captures} />
          </div>
        )}
      </div>

      {/* ── MODALS ── */}
      <MapModal />
      <QuestionModal />
    </div>
  );
}