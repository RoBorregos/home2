"use client";

import { useState } from "react";

interface MjpegStreamProps {
  streamUrl: string;
}

function buildSrc(url: string, retry: number) {
  const separator = url.includes("?") ? "&" : "?";
  return `${url}${separator}retry=${retry}`;
}

export default function MjpegStream({ streamUrl }: MjpegStreamProps) {
  const [retryCount, setRetryCount] = useState(0);

  const src = buildSrc(streamUrl, retryCount);

  const handleError = () => {
    setTimeout(() => {
      setRetryCount((prev) => prev + 1);
    }, 2000);
  };

  return (
    <div className="w-full h-full flex items-center justify-center">
      {/* eslint-disable-next-line @next/next/no-img-element */}
      <img
        src={src}
        alt="MJPEG Stream"
        onError={handleError}
        className="w-full h-full object-contain rounded-lg border border-(--border-light)"
      />
    </div>
  );
}
