"use client";

import React, { JSX } from "react";
import dynamic from "next/dynamic";

const Camera = dynamic(() => import("./video"), { ssr: false });


const Microphone: React.FC = () => (
  <div className="flex items-center justify-center">
    <div className="w-12 h-12 bg-gray-100 rounded-full flex items-center justify-center text-gray-600">🎤</div>
  </div>
);

const Messages: React.FC = () => (
  <div className="text-sm text-gray-700">
    <div className="mb-1 font-medium">Messages</div>
    <div className="space-y-2 max-h-40 overflow-auto">
      <div className="text-xs text-gray-500">No messages</div>
    </div>
  </div>
);

const StartButton: React.FC<{ onClick?: () => void }> = ({ onClick }) => (
  <button
    onClick={onClick}
    className="px-4 py-2 bg-blue-600 text-white rounded-md shadow-sm hover:bg-blue-700"
  >
    Start
  </button>
);

export default function GPSRDisplay(): JSX.Element {
  const defaultTopic = "/zed/zed_node/rgb/image_rect_color";

  return (
    <div className="flex flex-col h-screen bg-gray-50">
      <header className="flex items-center justify-between p-4 border-b bg-white/70">
        <h1 className="text-lg font-semibold">GPSR</h1>
        <div className="flex items-center gap-3">
          <div className="text-sm text-gray-600">Status: <span className="ml-1 font-medium text-green-600">Connected</span></div>
          <StartButton />
        </div>
      </header>

      <main className="flex-1 p-4">
        <div className="mx-auto max-w-7xl h-full grid grid-rows-[auto_1fr] gap-4">
          <section className="grid grid-cols-1 lg:grid-cols-6 gap-4 items-start h-full">
            <aside className="lg:col-span-1 flex lg:flex-col items-center justify-start gap-4">
              <div className="w-full max-w-xs">
                <div className="bg-white rounded-lg p-3 shadow">
                  <Microphone />
                </div>
              </div>
            </aside>

            <div className="lg:col-span-4 flex items-center justify-center">
              <div className="w-full max-w-4xl aspect-[16/9] bg-black rounded-lg overflow-hidden shadow-lg">
                {/* Camera component from ./video.tsx */}
                <Camera streamUrl={`http://localhost:8080/stream?topic=${encodeURIComponent(defaultTopic)}`} />
              </div>
            </div>

            <aside className="lg:col-span-1 flex flex-col items-center gap-4">
              <div className="w-full">
                <div className="bg-white rounded-lg p-3 shadow flex items-center justify-center">
                  <StartButton />
                </div>
              </div>
            </aside>
          </section>

          <section className="h-48">
            <div className="h-full bg-white rounded-lg p-3 shadow overflow-auto">
              <Messages />
            </div>
          </section>
        </div>
      </main>
    </div>
  );
}