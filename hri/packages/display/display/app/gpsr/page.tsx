"use client";

import { useEffect, useState } from "react";
import {
  Bot,
  Flame,
  Mic,
  Play,
  CheckCircle2,
  Navigation,
  Package,
  MessageSquare,
  MessageCircle,
  Eye,
  HandHelping,
  UserRound,
  ScanFace,
  BarChart3,
  Search,
} from "lucide-react";
import { Topic } from "roslib";
import { AudioStateIndicator } from "../../components/InteractionIndicator";
import { ConnectionStatus } from "../../components/ConnectionStatus";
import { MessagesList } from "../../components/MessagesList";
import { MapModal } from "../../components/MapModal";
import { QuestionModal } from "../../components/QuestionModal";
import { VideoFeed } from "../../components/VideoFeed";
import { StartButton } from "../../components/StartButton";
import { rosClient } from "../../RosClient";

type DisplayMode = "button" | "camera" | "logs" | "both";

interface FsmStep {
  key: string;
  label: string;
  icon: React.ReactNode;
}

const FSM_STEPS: FsmStep[] = [
  { key: "waiting_for_button",  label: "Wait Start",  icon: <Flame        className="h-3.5 w-3.5" /> },
  { key: "start",               label: "Start",       icon: <Bot          className="h-3.5 w-3.5" /> },
  { key: "wait_button_command", label: "Ready",       icon: <Flame        className="h-3.5 w-3.5" /> },
  { key: "waiting_for_command", label: "Listening",   icon: <Mic          className="h-3.5 w-3.5" /> },
  { key: "executing",           label: "Executing",   icon: <Play         className="h-3.5 w-3.5" /> },
  { key: "finished_command",    label: "Finished",    icon: <CheckCircle2 className="h-3.5 w-3.5" /> },
];

const COMMAND_DISPLAY: Record<string, DisplayMode> = {
  go_to:               "camera",
  pick_object:         "camera",
  place_object:        "camera",
  say_with_context:    "logs",
  say:                 "logs",
  answer_question:     "both",
  get_visual_info:     "camera",
  give_object:         "both",
  follow_person_until: "camera",
  guide_person_to:     "camera",
  get_person_info:     "camera",
  count:               "camera",
  find_person:         "camera",
  find_person_by_name: "camera",
};

interface CommandInfo {
  label: string;
  icon: React.ReactNode;
}

const COMMAND_INFO: Record<string, CommandInfo> = {
  go_to:               { label: "Go To",         icon: <Navigation    className="h-4 w-4" /> },
  pick_object:         { label: "Pick Object",   icon: <Package       className="h-4 w-4" /> },
  place_object:        { label: "Place Object",  icon: <Package       className="h-4 w-4" /> },
  say_with_context:    { label: "Say",           icon: <MessageSquare className="h-4 w-4" /> },
  say:                 { label: "Say",           icon: <MessageSquare className="h-4 w-4" /> },
  answer_question:     { label: "Answer",        icon: <MessageCircle className="h-4 w-4" /> },
  get_visual_info:     { label: "Visual Info",   icon: <Eye           className="h-4 w-4" /> },
  give_object:         { label: "Give Object",   icon: <HandHelping   className="h-4 w-4" /> },
  follow_person_until: { label: "Follow Person", icon: <UserRound     className="h-4 w-4" /> },
  guide_person_to:     { label: "Guide Person",  icon: <Navigation    className="h-4 w-4" /> },
  get_person_info:     { label: "Person Info",   icon: <ScanFace      className="h-4 w-4" /> },
  count:               { label: "Count",         icon: <BarChart3     className="h-4 w-4" /> },
  find_person:         { label: "Find Person",   icon: <Search        className="h-4 w-4" /> },
  find_person_by_name: { label: "Find by Name",  icon: <ScanFace      className="h-4 w-4" /> },
};

function parseFsmKey(raw: string): { fsmState: string; command: string | null } {
  if (raw.startsWith("executing:")) {
    return { fsmState: "executing", command: raw.slice("executing:".length) };
  }
  return { fsmState: raw, command: null };
}

function getDisplayMode(fsmState: string, command: string | null): DisplayMode {
  if (fsmState === "waiting_for_button" || fsmState === "wait_button_command") return "button";
  if (fsmState === "start")                                                     return "camera";
  if (fsmState === "waiting_for_command")                                       return "logs";
  if (fsmState === "finished_command" || fsmState === "done")                   return "logs";
  if (fsmState === "executing" && command) return COMMAND_DISPLAY[command] ?? "camera";
  return "camera";
}

function StepPill({
  step,
  state,
}: {
  step: FsmStep;
  state: "done" | "active" | "pending";
}) {
  const base =
    "flex items-center gap-1.5 px-2.5 py-1 rounded-full text-xs font-medium transition-all duration-300 whitespace-nowrap";
  const styles = {
    done:    "bg-emerald-500/20 text-emerald-400",
    active:  "bg-(--blue-bg) text-(--blue) ring-1 ring-(--blue)/40 shadow-md shadow-(--blue)/10 animate-pulse",
    pending: "bg-white/5 text-(--text-gray)/60",
  };
  return (
    <div className={`${base} ${styles[state]}`}>
      {step.icon}
      <span className="hidden lg:inline">{step.label}</span>
    </div>
  );
}

function CommandBadge({ command }: { command: string | null }) {
  if (!command) return null;
  const info = COMMAND_INFO[command];
  if (!info) return null;
  return (
    <div className="flex items-center gap-1.5 px-3 py-1 rounded-full text-xs font-semibold bg-amber-500/15 text-amber-400 ring-1 ring-amber-500/30 whitespace-nowrap">
      {info.icon}
      <span>{info.label}</span>
    </div>
  );
}

function CommandCounter({ index, max = 3 }: { index: number; max?: number }) {
  return (
    <div className="flex items-center gap-1.5 shrink-0">
      <span className="text-[10px] text-(--text-gray) mr-0.5 uppercase tracking-wide">cmd</span>
      {Array.from({ length: max }, (_, i) => (
        <div
          key={i}
          className={`flex items-center justify-center h-5 w-5 rounded-full text-[10px] font-bold transition-all duration-300 ${
            i < index
              ? "bg-emerald-500/20 text-emerald-400"
              : i === index
                ? "bg-(--blue-bg) text-(--blue) ring-1 ring-(--blue)/40"
                : "bg-white/5 text-(--text-gray)/40"
          }`}
        >
          {i + 1}
        </div>
      ))}
    </div>
  );
}

export default function GPSRPage() {
  const [taskStep,      setTaskStep]      = useState<string>("waiting_for_button");
  const [commandIndex,  setCommandIndex]  = useState<number>(0);

  useEffect(() => {
    const stepTopic = new Topic<{ data: string }>({
      ros:         rosClient,
      name:        "/gpsr/display/task_step",
      messageType: "std_msgs/String",
    });
    stepTopic.subscribe((msg) => {
      setTaskStep(msg.data.trim().toLowerCase());
    });

    const idxTopic = new Topic<{ data: number }>({
      ros:         rosClient,
      name:        "/gpsr/display/command_index",
      messageType: "std_msgs/Int32",
    });
    idxTopic.subscribe((msg) => {
      setCommandIndex(msg.data);
    });

    return () => {
      stepTopic.unsubscribe();
      idxTopic.unsubscribe();
    };
  }, []);

  const { fsmState, command } = parseFsmKey(taskStep);
  const displayMode            = getDisplayMode(fsmState, command);
  const activeFsmIndex         = FSM_STEPS.findIndex((s) => s.key === fsmState);

  return (
    <div className="flex flex-col h-screen bg-(--bg-dark) text-(--text-light) overflow-hidden">
      {/* ── HEADER ── */}
      <div className="p-3 border-b border-(--border-light) flex items-center justify-between shrink-0">
        <div className="flex items-center gap-3">
          <h1 className="text-lg font-bold flex items-center gap-2">
            <Bot className="h-5 w-5" />
            GPSR
          </h1>
          <CommandBadge command={command} />
        </div>
        <div className="flex items-center gap-4">
          <CommandCounter index={commandIndex} />
          <AudioStateIndicator />
          <ConnectionStatus />
        </div>
      </div>

      {/* ── FSM PROGRESS BAR ── */}
      <div className="px-3 py-2 border-b border-(--border-light) shrink-0 overflow-x-auto">
        <div className="flex items-center gap-1.5">
          {FSM_STEPS.map((step, i) => (
            <StepPill
              key={step.key}
              step={step}
              state={
                i < activeFsmIndex ? "done" : i === activeFsmIndex ? "active" : "pending"
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

        {/* CAMERA mode: full-width video feed */}
        {displayMode === "camera" && (
          <div className="h-full flex items-center justify-center p-4">
            <VideoFeed />
          </div>
        )}

        {/* LOGS mode: full-width messages */}
        {displayMode === "logs" && (
          <div className="h-full overflow-y-auto">
            <MessagesList />
          </div>
        )}

        {/* BOTH mode: messages + camera side by side */}
        {displayMode === "both" && (
          <div className="grid grid-cols-2 h-full overflow-hidden">
            <div className="border-r border-(--border-light) overflow-y-auto">
              <MessagesList />
            </div>
            <div className="flex items-center justify-center p-4">
              <VideoFeed />
            </div>
          </div>
        )}
      </div>

      {/* ── MODALS ── */}
      <MapModal />
      <QuestionModal />
    </div>
  );
}
