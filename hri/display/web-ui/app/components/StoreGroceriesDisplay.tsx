"use client";

import React from "react";
import dynamic from "next/dynamic";

const Camera = dynamic(() => import("./video"), { ssr: false });

// Local StartButton fallback (uses same look as GPSR's StartButton placeholder)
const StartButton: React.FC<{ onClick?: () => void }> = ({ onClick }) => (
  <button
    onClick={onClick}
    className="px-5 py-3 bg-blue-600 text-white rounded-full shadow-lg hover:bg-blue-700 focus:outline-none"
    aria-label="Start Store Groceries Task"
  >
    Start
  </button>
);

export default function StoreGroceriesDisplay(): React.ReactElement {
  const defaultTopic = "/zed/zed_node/rgb/image_rect_color";

  return (
    <div className="relative h-screen w-screen bg-black">
      {/* Full-bleed camera */}
      <div className="h-full w-full">
        <div className="h-full w-full overflow-hidden">
          <Camera streamUrl={`http://localhost:8080/stream?topic=${encodeURIComponent(defaultTopic)}`} />
        </div>
      </div>

      {/* Fixed bottom-center start button */}
      <div className="absolute left-1/2 bottom-8 transform -translate-x-1/2 z-30">
        <StartButton />
      </div>

      {/* Optional small header / status */}
      <div className="absolute top-4 left-4 z-30">
        <div className="bg-white/80 text-sm text-gray-800 px-3 py-1 rounded-md shadow">Store Groceries</div>
      </div>
    </div>
  );
}