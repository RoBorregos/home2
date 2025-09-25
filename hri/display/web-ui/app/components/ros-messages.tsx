"use client";

import { useEffect, useState, useRef } from "react";
import { MessageCircle } from "lucide-react";
import dynamic from "next/dynamic";

import { Message, AudioState, MapData, mapSchema } from "./types";
import { AudioStateIndicator } from "./AudioStateIndicator";
import { MapModal } from "./MapModal";
import { QuestionModal } from "./QuestionModal";
import { MessageItem } from "./MessageItem";
import { CurrentMessageOverlay } from "./CurrentMessageOverlay";
import { useWebSocket } from "./useWebSocket";

const MjpegStream = dynamic(() => import("./video"), { ssr: false });

export default function RosMessagesDisplay() {
  const [messages, setMessages] = useState<Message[]>([]);
  const [currentMessage, setCurrentMessage] = useState<Message | null>(null);
  const [audioState, setAudioState] = useState<AudioState>({
    state: "idle",
    vadLevel: 0,
  });
  const [mapData, setMapData] = useState<MapData | null>(null);
  const [showMapModal, setShowMapModal] = useState(false);
  const audioStateRef = useRef(audioState);
  useEffect(() => {
    audioStateRef.current = audioState;
  }, [audioState]);

  const [question, setQuestion] = useState<string | null>(null);
  const [answer, setAnswer] = useState<string>("");
  const [modalOpen, setModalOpen] = useState(false);
  
  const [audioTopic, setAudioTopic] = useState<string>(
    "/zed/zed_node/rgb/image_rect_color"
  );
  const messagesStartRef = useRef<HTMLDivElement>(null);
  const currentMessageTimeoutRef = useRef<NodeJS.Timeout | null>(null);

  // Add timeout effect for currentMessage
  useEffect(() => {
    // Clear any existing timeout
    if (currentMessageTimeoutRef.current) {
      clearTimeout(currentMessageTimeoutRef.current);
    }

    // If there's a current message and we're listening, set a 5-second timeout
    if (currentMessage && audioState.state === "listening") {
      currentMessageTimeoutRef.current = setTimeout(() => {
        setAudioState(prev => ({ ...prev, state: "idle" }));
      }, 5000);
    }

    // Cleanup function
    return () => {
      if (currentMessageTimeoutRef.current) {
        clearTimeout(currentMessageTimeoutRef.current);
      }
    };
  }, [currentMessage, audioState.state]);

  const addMessage = (
    type: "heard" | "spoken" | "keyword" | "answer" | "user_message" | "text_spoken",
    content: string
  ) => {
    let displayContent = content;

    if (type === "keyword") {
      try {
        // Replace single quotes with double quotes to make valid JSON
        const jsonString = content.replace(/'/g, '"');
        const parsedContent = JSON.parse(jsonString);

        if (parsedContent.score !== -1) {
          displayContent = parsedContent.keyword;
        } else {
          return;
        }
      } catch (error) {
        console.error("Error parsing keyword content:", error);
      }
    }
    const timestamp = new Date();
    const newMessage: Message = { type, content: displayContent, timestamp };

  if (type === "heard") {
    console.log("Heard message received ", audioStateRef.current.state);
    if (audioStateRef.current.state === "listening") {
      setCurrentMessage(newMessage);
    } else {
      setMessages((prev) => [newMessage, ...prev]);
  }
} else {
  setMessages(prev => [newMessage, ...prev]);
}
};

 const handleMic = (type: string, content: string | number) => {
  console.log("handleMic received:", type, content);
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

const handleMapMessage = (data: unknown) => {
  try {
    const parsedData = mapSchema.parse(data);
    // Only show the map if image_path is available
    if (parsedData.image_path && parsedData.image_path.trim() !== "") {
      setMapData(parsedData);
      setShowMapModal(true);
    } else {
      setMapData(null);
      setShowMapModal(false);
    }
  } catch (error) {
    console.error("Error parsing map message:", error);
  }
};

useEffect(() => {
  if (audioState.state === "idle" && currentMessage) {
    setMessages(prevMsgs => [currentMessage, ...prevMsgs]);
    setCurrentMessage(null);
  }
}, [audioState.state, currentMessage]);

useEffect(() => {
  messagesStartRef.current?.scrollIntoView({ behavior: "smooth" });
}, [messages]);

const { connected, sendMessage } = useWebSocket({
  onAddMessage: addMessage,
  onAudioStateChange: handleMic,
  onVideoTopicChange: setAudioTopic,
  onQuestionReceived: (question: string) => {
    if (!question?.trim()) {
      // If question is empty, close the modal and reset state
      setModalOpen(false);
      setQuestion(null);
      setAnswer("");
    } else {
      setModalOpen(true);
      setQuestion(question);
      addMessage("spoken", question);
    }
  },
  onMapReceived: handleMapMessage,
});

const sendAnswer = () => {
  if (!answer.trim()) return;
  addMessage("answer", answer);

  // Send to WebSocket server
  sendMessage({ 
    type: "answer", 
    answer: answer,
    isLocal: true 
  });

  // Clear the state
  setModalOpen(false);
  setAnswer("");
};
  return (
    <div className="flex flex-col h-screen bg-[oklch(0.145_0_0)] text-[oklch(0.985_0_0)] overflow-y-hidden">
      <div className="p-4 border-b border-[oklch(1_0_0/10%)] flex items-center justify-between">
        <h1 className="text-xl font-bold flex items-center">
          <MessageCircle className="mr-2 h-5 w-5" />
          ROS2 Messages
        </h1>
        <div className="flex items-center gap-3">
          <AudioStateIndicator state={audioState} />
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
      <div className="grid grid-cols-2 h-full overflow-y-hidden">
        {/* Left column */}
        <div className="flex flex-col p-4 space-y-3 overflow-y-auto">
          <div ref={messagesStartRef} />
          {messages.length === 0 ? (
            <div className="flex items-center justify-center h-full text-[oklch(0.708_0_0)]">
              <p>Waiting for messages...</p>
            </div>
          ) : (
            messages.map((msg, index) => (
              <MessageItem 
                key={index} 
                message={msg} 
                isLatest={index === 0} 
              />
            ))
          )}
        </div>
        {/* Right column */}
        <div className="sticky top-0 self-start h-[inherit] border-l border-[oklch(1_0_0/10%)] bg-[oklch(0.145_0_0)]">
          <div className="h-full flex flex-col items-center justify-center p-4">
            <button className="
              mb-4 px-4 py-2 bg-[oklch(0.488_0.243_264.376)] text-white rounded-lg hover:bg-[oklch(0.488_0.243_264.376/80%)] transition-colors w-3/4 h-16 text-lg font-semibold "
             onClick={() => void fetch("http://localhost:8001/send_button_press")}>
              Start 🔥
            </button>
            <p className="text-xl mb-4">Video feed at {audioTopic}</p>
            <MjpegStream
              streamUrl={`http://localhost:8080/stream?topic=${audioTopic}`}
            />
          </div>
        </div>
      </div>

      <div className="p-3 border-t border-[oklch(1_0_0/10%)] bg-[oklch(0.205_0_0/50%)] text-center">
        <p className="text-sm text-[oklch(0.708_0_0)]">
          {messages.length > 0
            ? `${messages.length} message${
                messages.length === 1 ? "" : "s"
              } received`
            : "No messages received"}
        </p>
      </div>

      {/* Map Modal */}
      {showMapModal && mapData && (
        <MapModal
          mapData={mapData}
          onClose={() => setShowMapModal(false)}
        />
      )}
      {modalOpen && question && (
        <QuestionModal
          question={question}
          answer={answer}
          onAnswerChange={setAnswer}
          onSendAnswer={sendAnswer}
          onClose={() => {
            setModalOpen(false);
            setQuestion(null);
            setAnswer("");
          }}
        />
      )}

    </div>
  );
}



