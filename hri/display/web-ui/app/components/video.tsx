"use client";

import { useEffect, useRef, useState } from "react";

type MjpegStreamProps = {
  streamUrl: string; // e.g. http://localhost:8080/stream?topic=/camera/image_raw
};

export default function MjpegStream({ streamUrl }: MjpegStreamProps) {
  const imgRef = useRef<HTMLImageElement>(null);
  const [retryKey, setRetryKey] = useState(Date.now());

  useEffect(() => {
    const tryReload = () => {
      setRetryKey(Date.now()); // force reload with new URL to bust cache
    };

    const interval = setInterval(() => {
      if (imgRef.current && !imgRef.current.complete) {
        tryReload();
      }
    }, 1000); // retry every 1s if not loaded

    return () => clearInterval(interval);
  }, []);

  const fullUrl = `${streamUrl}&rand=${retryKey}`;

  return (
    <img
      ref={imgRef}
      src={fullUrl}
      alt="Live stream"
      onError={() => {
        console.warn("Stream not available, will retry...");
      }}
      onLoad={() => {
        console.log("Stream loaded successfully");
      }}
      style={{ width: "100%", maxWidth: "640px", borderRadius: "8px" }}
    />
  );
}
