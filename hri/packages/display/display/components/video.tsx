"use client";

import { useEffect, useRef, useState } from "react";

interface MjpegStreamProps {
  streamUrl: string;
}

export default function MjpegStream({ streamUrl }: MjpegStreamProps) {
  const [retryCount, setRetryCount] = useState(0);
  const imgRef = useRef<HTMLImageElement>(null);

  useEffect(() => {
    if (imgRef.current) {
      const separator = streamUrl.includes("?") ? "&" : "?";
      imgRef.current.src = `${streamUrl}${separator}retry=${retryCount}`;
    }
  }, [streamUrl, retryCount]);

  const handleError = () => {
    setTimeout(() => {
      setRetryCount((prev) => prev + 1);
    }, 2000);
  };

  return (
    <div className="w-full max-w-3xl">
      <img
        key={retryCount}
        ref={imgRef}
        alt="MJPEG Stream"
        onError={handleError}
        className="w-full h-auto rounded-lg border border-(--border-light)"
      />
    </div>
  );
}

