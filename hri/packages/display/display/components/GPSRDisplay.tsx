"use client";

import { useEffect, useState, useRef } from "react";
import { MessageCircle } from "lucide-react";
import dynamic from "next/dynamic";
import { Topic } from "roslib";
import { rosClient } from "../RosClient";

import { Message, AudioState } from "../types";
import { AudioStateIndicator } from "./AudioStateIndicator";
import { MapModal } from "./MapModal";
import { QuestionModal } from "./QuestionModal";
import { MessageItem } from "./MessageItem";
import { CurrentMessageOverlay } from "./CurrentMessageOverlay";

const MjpegStream = dynamic(() => import("./video"), { ssr: false });

export default function GPSRDisplay() {
  const [messages, setMessages] = useState<Message[]>([]);
  const [currentMessage, setCurrentMessage] = useState<Message | null>(null);
  const [audioState, setAudioState] = useState<AudioState>({
    state: "idle",
    vadLevel: 0,
  });

  // We don't need mapData state here anymore as MapModal handles it

  const audioStateRef = useRef(audioState);

  useEffect(() => {
    audioStateRef.current = audioState;
  }, [audioState]);

  const [videoTopic, setVideoTopic] = useState<string>(
    "/zed/zed_node/rgb/image_rect_color"
  );
  const messagesStartRef = useRef<HTMLDivElement>(null);
  const currentMessageTimeoutRef = useRef<NodeJS.Timeout | null>(null);

  // Timeout for currentMessage overlay
  useEffect(() => {
    if (currentMessageTimeoutRef.current) clearTimeout(currentMessageTimeoutRef.current);

    if (currentMessage && audioState.state === "listening") {
      currentMessageTimeoutRef.current = setTimeout(() => {
        setAudioState(prev => ({ ...prev, state: "idle" }));
      }, 5000);
    }

    return () => {
      if (currentMessageTimeoutRef.current) clearTimeout(currentMessageTimeoutRef.current);
    };
  }, [currentMessage, audioState.state]);

  const addMessage = (
    type: "heard" | "spoken" | "keyword" | "answer" | "user_message" | "text_spoken",
    content: string
  ) => {
    let displayContent = content;

    if (type === "keyword") {
      try {
        const jsonString = content.replace(/'/g, '"');
        const parsedContent = JSON.parse(jsonString);
        if (parsedContent.score !== -1) displayContent = parsedContent.keyword;
        else return;
      } catch (error) {
        console.error("Error parsing keyword content:", error);
      }
    }

    const timestamp = new Date();
    const newMessage: Message = { type, content: displayContent, timestamp };

    if (type === "heard") {
      if (audioStateRef.current.state === "listening") {
        setCurrentMessage(newMessage);
      } else {
        setMessages(prev => [newMessage, ...prev]);
      }
    } else {
      setMessages(prev => [newMessage, ...prev]);
    }
  };

  const handleMic = (type: string, content: string | number) => {
    if (type === "audioState") {
      const newState = content as "idle" | "listening" | "saying";
      setAudioState(prev => ({ ...prev, state: newState }));
    } else if (type === "vad") {
      setAudioState(prev => ({
        ...prev,
        vadLevel: typeof content === "number" ? content : Number.parseFloat(content as string),
      }));
    }
  };

  useEffect(() => {
    if (audioState.state === "idle" && currentMessage) {
      setMessages(prev => [currentMessage, ...prev]);
      setCurrentMessage(null);
    }
  }, [audioState.state, currentMessage]);

  useEffect(() => {
    messagesStartRef.current?.scrollIntoView({ behavior: "smooth" });
  }, [messages]);

  const [connected, setConnected] = useState(false);

  useEffect(() => {
    // Connection handling
    const handleConnection = () => setConnected(true);
    const handleClose = () => setConnected(false);
    const handleError = () => setConnected(false);

    rosClient.on("connection", handleConnection);
    rosClient.on("close", handleClose);
    rosClient.on("error", handleError);
    if (rosClient.isConnected) setConnected(true);

    // Topics
    const audioStateTopic = new Topic<{ data: string }>({
      ros: rosClient,
      name: "/AudioState",
      messageType: "std_msgs/String",
    });

    const rawCommandTopic = new Topic<{ data: string }>({
      ros: rosClient,
      name: "/speech/raw_command",
      messageType: "std_msgs/String",
    });

    const textSpokenTopic = new Topic<{ data: string }>({
      ros: rosClient,
      name: "/speech/text_spoken",
      messageType: "std_msgs/String",
    });

    const owwTopic = new Topic<{ data: string }>({
      ros: rosClient,
      name: "/hri/speech/oww",
      messageType: "std_msgs/String",
    });

    const vadTopic = new Topic<{ data: number }>({
      ros: rosClient,
      name: "/hri/speech/vad",
      messageType: "std_msgs/Float32",
    });

    const answersTopic = new Topic<{ data: string }>({
      ros: rosClient,
      name: "/hri/display/answers",
      messageType: "std_msgs/String",
    });

    const videoTopic = new Topic<{ data: string }>({
      ros: rosClient,
      name: "/hri/display/change_video",
      messageType: "std_msgs/String",
    });

    // We subscribe to questions just to log them, QuestionModal handles display
    const questionsTopic = new Topic<{ data: string }>({
      ros: rosClient,
      name: "/hri/display/frida_questions",
      messageType: "std_msgs/String",
    });

    // Subscriptions
    audioStateTopic.subscribe((msg) => handleMic("audioState", msg.data));
    rawCommandTopic.subscribe((msg) => addMessage("heard", msg.data));
    textSpokenTopic.subscribe((msg) => addMessage("spoken", msg.data));
    owwTopic.subscribe((msg) => addMessage("keyword", msg.data));
    vadTopic.subscribe((msg) => handleMic("vad", msg.data));
    answersTopic.subscribe((msg) => addMessage("answer", msg.data));
    videoTopic.subscribe((msg) => setVideoTopic(msg.data));

    questionsTopic.subscribe((msg) => {
      const question = msg.data;
      if (question?.trim()) {
        addMessage("spoken", question);
      }
    });

    return () => {
      rosClient.off("connection", handleConnection);
      rosClient.off("close", handleClose);
      rosClient.off("error", handleError);

      audioStateTopic.unsubscribe();
      rawCommandTopic.unsubscribe();
      textSpokenTopic.unsubscribe();
      owwTopic.unsubscribe();
      vadTopic.unsubscribe();
      answersTopic.unsubscribe();
      videoTopic.unsubscribe();
      questionsTopic.unsubscribe();
    };
  }, []);

  return (
    <div className="flex flex-col h-screen bg-[oklch(0.145_0_0)] text-[oklch(0.985_0_0)] overflow-y-hidden">

      {/* HEADER */}
      <div className="p-4 border-b border-[oklch(1_0_0/10%)] flex items-center justify-between">
        <h1 className="text-xl font-bold flex items-center">
          <MessageCircle className="mr-2 h-5 w-5" />
          GPSR Commands
        </h1>
        <div className="flex items-center gap-3">
          <AudioStateIndicator />
          <div
            className={
              connected
                ? "px-2 py-1 rounded-full text-xs font-medium bg-[oklch(0.488_0.243_264.376/20%)] text-[oklch(0.488_0.243_264.376)]"
                : "px-2 py-1 rounded-full text-xs font-medium bg-[oklch(0.577_0.245_27.325/20%)] text-[oklch(0.704_0.191_22.216)]"
            }
          >
            {connected ? "Connected" : "Disconnected"}
          </div>
        </div>
      </div>

      {audioState.state === "listening" && currentMessage && (
        <CurrentMessageOverlay message={currentMessage} />
      )}

      {/* GRID */}
      <div className="grid grid-cols-3 h-full overflow-y-hidden">
        {/* Left column - Messages */}
        <div className="flex flex-col p-4 space-y-3 overflow-y-auto col-span-1">
          <div ref={messagesStartRef} />
          {messages.length === 0 ? (
            <div className="flex items-center justify-center h-full text-[oklch(0.708_0_0)]">
              <p>Waiting for commands...</p>
            </div>
          ) : (
            messages.map((msg, index) => (
              <MessageItem key={index} message={msg} isLatest={index === 0} />
            ))
          )}
        </div>

        {/* Center & Right column - Video + Start */}
        <div className="col-span-2 sticky top-0 self-start h-[inherit] border-l border-[oklch(1_0_0/10%)] bg-[oklch(0.145_0_0)]">
          <div className="h-full flex flex-col items-center justify-center p-4">
            <button
              className="mb-4 px-4 py-2 bg-[oklch(0.488_0.243_264.376)] text-white rounded-lg hover:bg-[oklch(0.488_0.243_264.376/80%)] transition-colors w-3/4 h-16 text-lg font-semibold"
              onClick={() => void fetch("http://localhost:8001/send_button_press")}
            >
              Start 🔥
            </button>
            <p className="text-xl mb-4">Video feed at {videoTopic}</p>
            <div className="w-full max-w-3xl rounded-xl overflow-hidden shadow-xl border border-[oklch(1_0_0/15%)]">
              <MjpegStream streamUrl={`http://localhost:8080/stream?topic=${videoTopic}`} />
            </div>
          </div>
        </div>
      </div>

      {/* FOOTER */}
      <div className="p-3 border-t border-[oklch(1_0_0/10%)] bg-[oklch(0.205_0_0/50%)] text-center">
        <p className="text-sm text-[oklch(0.708_0_0)]">
          {messages.length > 0
            ? `${messages.length} command${messages.length === 1 ? "" : "s"} received`
            : "No commands received"}
        </p>
      </div>

      {/* Map Modal - Self-contained */}
      <MapModal />

      {/* Question Modal - Self-contained */}
      <QuestionModal />
    </div>
  );
}