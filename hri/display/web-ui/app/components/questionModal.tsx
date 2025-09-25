"use client";

interface QuestionModalProps {
  question: string;
  answer: string;
  onAnswerChange: (answer: string) => void;
  onSendAnswer: () => void;
  onClose: () => void;
}

export function QuestionModal({ 
  question, 
  answer, 
  onAnswerChange, 
  onSendAnswer, 
  onClose 
}: QuestionModalProps) {
  const handleKeyDown = (e: React.KeyboardEvent) => {
    if (e.key === "Enter" && e.shiftKey) {
      // Allow new lines with Shift+Enter
      return;
    }
    if (e.key === "Enter") {
      e.preventDefault(); // Prevent default Enter behavior
      onSendAnswer(); // Send answer on Enter
    }
  };

  return (
    <div className="fixed inset-0 z-70 flex items-center justify-center bg-black/50">
      <div 
        className="bg-white rounded-lg p-6 w-11/12 max-w-md"
        onClick={(e) => e.stopPropagation()} // Prevent click from closing modal
      >
        <h2 className="text-xl font-bold mb-4 text-black">Question</h2>
        <p className="mb-4 text-black">{question}</p>
        <textarea
          value={answer}
          onChange={(e) => onAnswerChange(e.target.value)}
          onKeyDown={handleKeyDown}
          className="w-full h-24 p-2 border border-gray-300 rounded mb-4 text-gray-500"
          placeholder="Type your answer here... "
        />
        <button
          onClick={onSendAnswer}
          className="w-full bg-blue-500 text-white py-2 rounded hover:bg-blue-600 transition-colors"
        >
          Send Answer
        </button>
        <button
          onClick={onClose}
          className="mt-3 w-full bg-gray-300 text-gray-800 py-2 rounded hover:bg-gray-400 transition-colors"
        >
          Close
        </button>
      </div>
    </div>
  );
}
