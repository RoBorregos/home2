"use client";

import { useEffect, useState } from "react";
import { X } from "lucide-react";
import Image from "next/image";
import { MapModalProps } from "./types";

export function MapModal({ mapData, onClose }: MapModalProps) {
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
            <Image
              src={getImageSrc(mapData.image_path)}
              alt="Map"
              width={800}
              height={600}
              className={`max-w-full max-h-full object-contain ${
                imageLoaded ? 'block' : 'hidden'
              }`}
              onLoad={() => setImageLoaded(true)}
              onError={() => setImageError(true)}
              unoptimized
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
