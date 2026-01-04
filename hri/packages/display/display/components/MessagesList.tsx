"use client";

import { useEffect, useState, useRef } from "react";
import { Topic } from "roslib";
import { rosClient } from "../RosClient";
import { Message } from "../types";
import { MessageItem } from "./MessageItem";
import { CurrentMessageOverlay } from "./CurrentMessageOverlay";

export function MessagesList() {
  const [messages, setMessages] = useState<Message[]>([]);
  const [currentMessage, setCurrentMessage] = useState<Message | null>(null);
  const [audioState, setAudioState] = useState<"idle" | "listening" | "saying">(
    "idle"
  );
  const messagesStartRef = useRef<HTMLDivElement>(null);
  const currentMessageTimeoutRef = useRef<NodeJS.Timeout | null>(null);
  const audioStateRef = useRef(audioState);

  useEffect(() => {
    audioStateRef.current = audioState;
  }, [audioState]);

  // Timeout effect for currentMessage
  useEffect(() => {
    if (currentMessageTimeoutRef.current) {
      clearTimeout(currentMessageTimeoutRef.current);
    }

    if (currentMessage && audioState === "listening") {
      currentMessageTimeoutRef.current = setTimeout(() => {
        setAudioState("idle");
      }, 5000);
    }

    return () => {
      if (currentMessageTimeoutRef.current) {
        clearTimeout(currentMessageTimeoutRef.current);
      }
    };
  }, [currentMessage, audioState]);

  // Move currentMessage to messages when state becomes idle
  useEffect(() => {
    if (audioState === "idle" && currentMessage) {
      setMessages((prevMsgs) => [currentMessage, ...prevMsgs]);
      setCurrentMessage(null);
    }
  }, [audioState, currentMessage]);

  // Auto-scroll
  useEffect(() => {
    messagesStartRef.current?.scrollIntoView({ behavior: "smooth" });
  }, [messages]);

  const addMessage = (
    type:
      | "heard"
      | "spoken"
      | "keyword"
      | "answer"
      | "user_message"
      | "text_spoken",
    content: string
  ) => {
    let displayContent = content;

    if (type === "keyword") {
      try {
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
      if (audioStateRef.current === "listening") {
        setCurrentMessage(newMessage);
      } else {
        setMessages((prev) => [newMessage, ...prev]);
      }
    } else {
      setMessages((prev) => [newMessage, ...prev]);
    }
  };

  useEffect(() => {
    // Subscribe to audio state
    const audioStateTopic = new Topic<{ data: string }>({
      ros: rosClient,
      name: "/AudioState",
      messageType: "std_msgs/String",
    });

    audioStateTopic.subscribe((msg: { data: string }) => {
      setAudioState(msg.data as "idle" | "listening" | "saying");
    });

    // Subscribe to raw command (heard)
    const rawCommandTopic = new Topic<{ data: string }>({
      ros: rosClient,
      name: "/speech/raw_command",
      messageType: "std_msgs/String",
    });

    rawCommandTopic.subscribe((msg: { data: string }) => {
      addMessage("heard", msg.data);
    });

    // Subscribe to text spoken
    const textSpokenTopic = new Topic<{ data: string }>({
      ros: rosClient,
      name: "/speech/text_spoken",
      messageType: "std_msgs/String",
    });

    textSpokenTopic.subscribe((msg: { data: string }) => {
      addMessage("spoken", msg.data);
    });

    // Subscribe to keyword (oww)
    const owwTopic = new Topic<{ data: string }>({
      ros: rosClient,
      name: "/hri/speech/oww",
      messageType: "std_msgs/String",
    });

    owwTopic.subscribe((msg: { data: string }) => {
      addMessage("keyword", msg.data);
    });

    // Subscribe to answers
    const answersTopic = new Topic<{ data: string }>({
      ros: rosClient,
      name: "/hri/display/answers",
      messageType: "std_msgs/String",
    });

    answersTopic.subscribe((msg: { data: string }) => {
      addMessage("answer", msg.data);
    });

    return () => {
      audioStateTopic.unsubscribe();
      rawCommandTopic.unsubscribe();
      textSpokenTopic.unsubscribe();
      owwTopic.unsubscribe();
      answersTopic.unsubscribe();
    };
  }, []);

  return (
    <>
      {audioState === "listening" && currentMessage && (
        <CurrentMessageOverlay message={currentMessage} />
      )}
      <div className="flex flex-col p-4 space-y-3 overflow-y-auto">
        <div ref={messagesStartRef} />
        {messages.length === 0 ? (
          <div className="flex items-center justify-center h-full text-(--text-gray)">
            <p>Waiting for messages...</p>
          </div>
        ) : (
          messages.map((msg, index) => (
            <MessageItem key={index} message={msg} isLatest={index === 0} />
          ))
        )}
      </div>
    </>
  );
}
