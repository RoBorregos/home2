"use client";

import { useEffect, useState } from "react";
import { Send, X } from "lucide-react";
import { Topic } from "roslib";
import { rosClient } from "../RosClient";

export function QuestionModal() {
  const [question, setQuestion] = useState<string | null>(null);
  const [answer, setAnswer] = useState<string>("");
  const [isOpen, setIsOpen] = useState(false);
  const [answerPublisher, setAnswerPublisher] = useState<Topic<{
    data: string;
  }> | null>(null);

  useEffect(() => {
    // Create publisher for answers
    const publisher = new Topic<{ data: string }>({
      ros: rosClient,
      name: "/hri/display/answers",
      messageType: "std_msgs/String",
    });
    setAnswerPublisher(publisher);

    // Subscribe to questions
    const questionTopic = new Topic<{ data: string }>({
      ros: rosClient,
      name: "/hri/display/frida_questions",
      messageType: "std_msgs/String",
    });

    questionTopic.subscribe((msg: { data: string }) => {
      if (!msg.data?.trim()) {
        setIsOpen(false);
        setQuestion(null);
        setAnswer("");
      } else {
        setQuestion(msg.data);
        setIsOpen(true);
      }
    });

    return () => {
      questionTopic.unsubscribe();
    };
  }, []);

  const handleSendAnswer = () => {
    if (!answer.trim() || !answerPublisher) return;

    // Publish answer to ROS topic
    answerPublisher.publish({ data: answer });

    // Clear and close
    setAnswer("");
    setIsOpen(false);
    setQuestion(null);
  };

  const handleClose = () => {
    setIsOpen(false);
    setQuestion(null);
    setAnswer("");
  };

  if (!isOpen || !question) return null;

  return (
    <div className="fixed inset-0 z-50 flex items-center justify-center bg-black/70 backdrop-blur-sm p-4">
      <div className="relative bg-(--bg-dark) rounded-xl shadow-2xl max-w-[95vw] md:max-w-2xl w-full p-4 md:p-6">
        <button
          onClick={handleClose}
          className="absolute top-3 right-3 md:top-4 md:right-4 p-1.5 md:p-2 rounded-full bg-(--bg-dark) hover:bg-(--blue-bg) transition-colors"
          aria-label="Close modal"
        >
          <X className="h-4 w-4 md:h-5 md:w-5 text-(--text-light)" />
        </button>

        <h2 className="text-xl md:text-2xl font-bold text-(--text-light) mb-3 md:mb-4">
          Question
        </h2>

        <div className="mb-4 md:mb-6 p-3 md:p-4 bg-(--bg-darker) rounded-lg">
          <p className="text-base md:text-lg text-(--text-light)">{question}</p>
        </div>

        <div className="space-y-3 md:space-y-4">
          <label className="block">
            <span className="text-xs md:text-sm font-medium text-(--text-light) mb-1.5 md:mb-2 block">
              Your Answer
            </span>
            <textarea
              value={answer}
              onChange={(e) => setAnswer(e.target.value)}
              placeholder="Type your answer here..."
              className="w-full px-3 md:px-4 py-2 md:py-3 bg-(--bg-darker) text-(--text-light) border border-(--border-light) rounded-lg focus:outline-none focus:ring-2 focus:ring-(--blue) resize-none text-sm md:text-base"
              rows={3}
              onKeyDown={(e) => {
                if (e.key === "Enter" && e.ctrlKey) {
                  handleSendAnswer();
                }
              }}
            />
          </label>

          <button
            onClick={handleSendAnswer}
            disabled={!answer.trim()}
            className="w-full flex items-center justify-center gap-2 px-4 md:px-6 py-2 md:py-3 bg-(--blue) hover:bg-(--blue-hover) disabled:bg-(--text-gray) disabled:cursor-not-allowed text-white font-semibold rounded-lg transition-colors text-sm md:text-base"
          >
            <Send className="h-4 w-4 md:h-5 md:w-5" />
            Send Answer (Ctrl+Enter)
          </button>
        </div>
      </div>
    </div>
  );
}
