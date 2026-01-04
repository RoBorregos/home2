"use client";

import { useEffect, useRef } from "react";

interface MjpegStreamProps {
  streamUrl: string;
}

export default function MjpegStream({ streamUrl }: MjpegStreamProps) {
  const imgRef = useRef<HTMLImageElement>(null);

  useEffect(() => {
    if (imgRef.current) {
      imgRef.current.src = streamUrl;
    }
  }, [streamUrl]);

  return (
    <div className="w-full max-w-3xl">
      <img
        ref={imgRef}
        alt="MJPEG Stream"
        className="w-full h-auto rounded-lg border border-(--border-light)"
      />
    </div>
  );
}
