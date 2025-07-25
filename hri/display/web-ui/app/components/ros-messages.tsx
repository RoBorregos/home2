"use client";

import { useEffect, useState, useRef } from "react";
import { MessageCircle, Mic, Speaker, Star, VolumeX, X } from "lucide-react";
import dynamic from "next/dynamic";
import { z } from "zod";

const MjpegStream = dynamic(() => import("./video"), { ssr: false });

const markerSchema = z.object({
  x: z.number(),
  y: z.number(),
  color: z.string(),
  color_name: z.string(),
});

const mapSchema = z.object({
  image_path: z.string(),
  markers: z.array(markerSchema),
});

type MapData = z.infer<typeof mapSchema>;

interface Message {
  type: "heard" | "spoken" | "keyword"| "answer" | "user_message" | "text_spoken";
  content: string;
  timestamp: Date;
}

interface AudioState {
  state: "idle" | "listening" | "saying";
  vadLevel: number;
}

export default function RosMessagesDisplay() {
  const [messages, setMessages] = useState<Message[]>([]);
  const [currentMessage, setCurrentMessage] = useState<Message | null>(null);
  const [connected, setConnected] = useState(false);
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
  const socketRef = useRef<WebSocket | null>(null);
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

  useEffect(() => {
    const socket = new WebSocket("ws://localhost:8001/");
    socketRef.current = socket;
    socket.onmessage = (event) => {
      const data = JSON.parse(event.data);
      if(data.type === "answer")return;
      if (data.type === "audioState" || data.type === "vad") {
        handleMic(data.type, data.data);
      } else if (data.type === "changeVideo") {
        setAudioTopic(data.data);
      } else if (data.type === "question") {
        if (!data.data?.trim()){
          // If question is empty, close the modal and reset state
          setModalOpen(false);
          setQuestion(null);
          setAnswer("");
        }else{
          setModalOpen(true);
          setQuestion(data.data);
          addMessage("spoken", data.data);
        }
      } else if (data.type === "map") {
        handleMapMessage(data.data);
      } else {
        addMessage(data.type, data.data);
      }
    };

    socket.onopen = () => {
      console.log("WebSocket connected");
      setConnected(true);
    };

    socket.onclose = () => {
      console.log("WebSocket disconnected");
      setConnected(false);
    };

    socket.onerror = (error) => {
      console.error("WebSocket error:", error);
      setConnected(false);
    };

    return () => {
      socket.close();
    };
  }, []);

  const addMessage = (
    type: "heard" | "spoken" | "keyword" | "answer" | "user_message",
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
const sendAnswer = () => {
  if (!answer.trim()) return;
  addMessage("answer", answer);

  // 2. Send to WebSocket server
  if (socketRef.current?.readyState === WebSocket.OPEN) {
    socketRef.current.send(
      JSON.stringify({ 
        type: "answer", 
        answer: answer,
        isLocal: true 
      })
    );
  }

  // 3. Clear the state
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
    <>
      <div
        key={currentMessage.timestamp.getTime()}
        className="fixed bottom-32 inset-x-0 mx-auto bg-gray-500/20 text-white py-6 rounded-lg shadow-lg z-50 animate-fadeIn w-fit max-w-[90vw]"
      >
        <div className="flex items-center justify-center gap-4">
          <Mic className="h-20 w-20 animate-pulse text-white" />
          <p className="text-7xl font-medium tracking-wide">{currentMessage.content}</p>
        </div>
      </div>
    </>
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
              <div
                key={index}
                className={`
                p-3 rounded-lg animate-fadeIn transition-all duration-300
                ${
                  msg.type === "heard"
                    ? "bg-[oklch(0.488_0.243_264.376/20%)] border-l-4 border-l-[oklch(0.488_0.243_264.376)]"
                    : msg.type === "spoken"
                    ? "bg-[oklch(0.627_0.265_303.9/20%)] border-l-4 border-l-[oklch(0.627_0.265_303.9)]"
                    : "bg-[oklch(0.9_0.3_60/20%)] border-l-4 border-l-[oklch(0.9_0.3_60)]"
                }
                ${
                  index === 0
                    ? "ring-2 ring-offset-2 ring-offset-[oklch(0.145_0_0)] ring-[oklch(0.985_0_0/10%)]"
                    : ""
                }
              `}
              >
                <div className="flex items-start gap-2">
                  {msg.type === "spoken" || msg.type === "text_spoken" ? (
                    <Speaker className="h-5 w-5 text-[oklch(0.627_0.265_303.9)] mt-0.5 flex-shrink-0" />
                  ) : msg.type === "answer" ? (
                    <MessageCircle className="h-5 w-5 text-[oklch(0.4_0.3_120)] mt-0.5 flex-shrink-0" />
                  ) : msg.type === "heard" ? (
                    <Mic className="h-5 w-5 text-[oklch(0.488_0.243_264.376)] mt-0.5 flex-shrink-0" />
                  ) : (
                    <Star className="h-5 w-5 text-[oklch(0.9_0.3_60)] mt-0.5 flex-shrink-0" />
                  )}
                  <div className="flex-1 min-w-0">
                    <p className="font-medium text-sm text-[oklch(0.708_0_0)]">
                      {msg.type === "spoken" || msg.type === "text_spoken"
                        ? "Spoken"
                        : msg.type === "answer"
                        ? "Answer"
                        : msg.type === "heard"
                        ? "Heard"
                        : "Keyword"}
                    </p>
                      <p className="text-lg font-bold break-words">
                      {msg.content}
                    </p>
                    <p className="text-xs text-[oklch(0.708_0_0)] mt-1">
                      {msg.timestamp.toLocaleTimeString([], {
                        hour: "2-digit",
                        minute: "2-digit",
                        second: "2-digit",
                      })}
                    </p>
                  </div>
                </div>
              </div>
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
        <div className="fixed inset-0 z-70 flex items-center justify-center bg-black/50">
          <div className="bg-white rounded-lg p-6 w-11/12 max-w-md"
            onClick={(e) => e.stopPropagation()} // Prevent click from closing modal
          >
            <h2 className="text-xl font-bold mb-4 text-black">Question</h2>
            <p className="mb-4 text-black">{question}</p>
            <textarea
              value={answer}
              onChange={(e) => setAnswer(e.target.value)}
              onKeyDown={(e) => {
              
                if (e.key === "Enter" && e.shiftKey) {
                  // Allow new lines with  // 1. Create and add the message locally
  const newMessage: Message = {
    type: "answer",
    content: answer,
    timestamp: new Date()
  };
  setMessages(prev => [newMessage, ...prev]);
                  return;
                }
                if (e.key === "Enter") {
                  e.preventDefault(); // Prevent default Enter behavior
                  sendAnswer(); // Send answer on Enter
                }
              }}
              className="w-full h-24 p-2 border border-gray-300 rounded mb-4 text-gray-500"
              placeholder="Type your answer here... "
            />
            <button
              onClick={sendAnswer}
              className="w-full bg-blue-500 text-white py-2 rounded hover:bg-blue-600 transition-colors"
            >
              Send Answer
            </button>
            <button
              onClick={() => {
                setModalOpen(false);
                setQuestion(null);
                setAnswer("");
              }}
              className="mt-3 w-full bg-gray-300 text-gray-800 py-2 rounded hover:bg-gray-400 transition-colors"
            >
              Close
            </button>
          </div>
        </div>
      )}

    </div>
  );
}

interface AudioStateIndicatorProps {
  state: AudioState;
}

function AudioStateIndicator({ state }: AudioStateIndicatorProps) {
  const { state: audioState, vadLevel } = state;

  // Calculate the number of bars to show based on VAD level (0-1)
  // const maxBars = 20;
  // const activeBars = Math.ceil(vadLevel * maxBars);

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

interface MapModalProps {
  mapData: MapData;
  onClose: () => void;
}

function MapModal({ mapData, onClose }: MapModalProps) {
  const [imageLoaded, setImageLoaded] = useState(false);
  const [imageError, setImageError] = useState(false);
  const [showAxes, setShowAxes] = useState(false);
  const [axesColor, setAxesColor] = useState<'white' | 'black'>('white');

  // Handle escape key to close modal, 'v' to toggle axes, and 'c' to toggle color
  useEffect(() => {
    const handleKeyDown = (event: KeyboardEvent) => {
      if (event.key === 'Escape') {
        onClose();
      }
      if (event.key === 'v' || event.key === 'V') {
        setShowAxes((prev) => !prev);
      }
      if (event.key === 'c' || event.key === 'C') {
        setAxesColor((prev) => prev === 'white' ? 'black' : 'white');
      }
    };

    document.addEventListener('keydown', handleKeyDown);
    return () => {
      document.removeEventListener('keydown', handleKeyDown);
    };
  }, [onClose]);

  // Determine if the image_path is a URL or a local filename
  const getImageSrc = (imagePath: string) => {
    // Check if it's already a full URL (starts with http:// or https://)
    if (imagePath.startsWith('http://') || imagePath.startsWith('https://')) {
      return imagePath;
    }
    // If it's a local filename, prepend with /
    return `/${imagePath}`;
  };

  // Generate grid lines at every 10%
  const gridLines = [];
  const lineColor = axesColor === 'white' ? 'border-white/40' : 'border-black/40';
  const textColor = axesColor === 'white' ? 'text-white/80' : 'text-black/80';
  const bgColor = axesColor === 'white' ? 'bg-black/60' : 'bg-white/60';
  
  for (let i = 10; i < 100; i += 10) {
    gridLines.push(
      // Vertical lines (X axis)
      <div
        key={`v-${i}`}
        className={`absolute top-0 left-0 h-full border-l-2 border-dashed ${lineColor} pointer-events-none`}
        style={{ left: `${i}%`, width: 0 }}
      >
        <span className={`absolute top-0 left-1/2 -translate-x-1/2 text-xs ${textColor} ${bgColor} px-1 rounded`}>
          X: {i}%
        </span>
      </div>,
      // Horizontal lines (Y axis)
      <div
        key={`h-${i}`}
        className={`absolute left-0 top-0 w-full border-t-2 border-dashed ${lineColor} pointer-events-none`}
        style={{ top: `${i}%`, height: 0 }}
      >
        <span className={`absolute left-0 top-1/2 -translate-y-1/2 text-xs ${textColor} ${bgColor} px-1 rounded`}>
          Y: {i}%
        </span>
      </div>
    );
  }
  
  return (
    <div className="fixed inset-0 z-50 bg-black/90 flex items-center justify-center">
      <div className="relative w-full h-full flex items-center justify-center p-4">
        {/* Close button */}
        <button
          onClick={onClose}
          className="absolute top-4 right-4 z-10 p-2 rounded-full bg-white/10 hover:bg-white/20 transition-colors"
        >
          <X className="h-6 w-6 text-white" />
        </button>

        {/* Map container */}
        <div className="relative max-w-full max-h-full">
          {!imageLoaded && !imageError && (
            <div className="flex items-center justify-center p-8">
              <div className="text-white text-lg">Loading map...</div>
            </div>
          )}

          {imageError && (
            <div className="flex items-center justify-center p-8">
              <div className="text-red-400 text-lg">Failed to load map image</div>
            </div>
          )}

          <div className="relative">
            <img
              src={getImageSrc(mapData.image_path)}
              alt="Map"
              className={`max-w-full max-h-full object-contain ${
                imageLoaded ? 'block' : 'hidden'
              }`}
              onLoad={() => setImageLoaded(true)}
              onError={() => setImageError(true)}
            />

            {/* Axes grid lines */}
            {imageLoaded && showAxes && (
              <div className="absolute inset-0 w-full h-full pointer-events-none z-20 border">
                {gridLines}
              </div>
            )}

            {/* Markers */}
            {imageLoaded && mapData.markers.map((marker, index) => {
              // Check if the marker is in the bottom half of the image
              const isInBottomHalf = marker.y > 75; // If marker is below 75% of image height
              
              return (
                <div
                  key={index}
                  className="absolute -translate-x-1/2 -translate-y-1/2 z-30"
                  style={{
                    left: `${marker.x}%`,
                    top: `${marker.y}%`,
                  }}
                >
                  {/* Marker circle */}
                  <div
                    className="w-20 h-20 rounded-full border-2 border-white shadow-lg"
                    style={{
                      backgroundColor: marker.color,
                    }}
                  />
                  {/* Color name label */}
                  <div 
                    className={`absolute left-1/2 -translate-x-1/2 whitespace-nowrap ${
                      isInBottomHalf ? 'bottom-24' : 'top-24'
                    }`}
                  >
                    <span className="bg-black/70 text-white px-2 py-1 rounded text-4xl font-medium">
                      {marker.color_name}
                    </span>
                  </div>
                </div>
              );
            })}
          </div>
        </div>

        {/* Instructions */}
        <div className="absolute bottom-4 left-1/2 -translate-x-1/2">
          <div className="bg-black/50 text-white px-4 py-2 rounded-lg text-sm">
            Press <b>ESC</b> or click the X to close. Press <b>V</b> to toggle axes. Press <b>C</b> to change axes color.
          </div>
        </div>
      </div>
    </div>
  );
}