"use client";

import { useEffect, useState } from "react";
import { X } from "lucide-react";
import { Topic } from "roslib";
import { rosClient } from "../RosClient";
import { MapData, mapSchema } from "../types";

export function MapModal() {
  const [mapData, setMapData] = useState<MapData | null>(null);
  const [isOpen, setIsOpen] = useState(false);

  useEffect(() => {
    const mapTopic = new Topic<{ data: string }>({
      ros: rosClient,
      name: "/hri/display/map",
      messageType: "std_msgs/String",
    });

    mapTopic.subscribe((msg: { data: string }) => {
      try {
        const parsedData = mapSchema.parse(JSON.parse(msg.data));
        // Only show the map if image_path is available
        if (parsedData.image_path && parsedData.image_path.trim() !== "") {
          setMapData(parsedData);
          setIsOpen(true);
        } else {
          setMapData(null);
          setIsOpen(false);
        }
      } catch (error) {
        console.error("Error parsing map message:", error);
      }
    });

    return () => {
      mapTopic.unsubscribe();
    };
  }, []);

  if (!isOpen || !mapData) return null;

  return (
    <div className="fixed inset-0 z-50 flex items-center justify-center bg-black/70 backdrop-blur-sm">
      <div className="relative bg-(--bg-dark) rounded-xl shadow-2xl max-w-4xl max-h-[90vh] overflow-auto p-6">
        <button
          onClick={() => setIsOpen(false)}
          className="absolute top-4 right-4 p-2 rounded-full bg-(--bg-darker) hover:bg-(--blue-bg) transition-colors"
          aria-label="Close modal"
        >
          <X className="h-5 w-5 text-(--text-light)" />
        </button>

        <h2 className="text-2xl font-bold text-(--text-light) mb-4">
          Map View
        </h2>

        <div className="relative">
          <img
            src={mapData.image_path}
            alt="Map"
            className="w-full h-auto rounded-lg"
          />
          {mapData.markers.map((marker, index) => (
            <div
              key={index}
              className="absolute w-4 h-4 rounded-full border-2 border-white shadow-lg"
              style={{
                left: `${marker.x}%`,
                top: `${marker.y}%`,
                backgroundColor: marker.color,
                transform: "translate(-50%, -50%)",
              }}
              title={marker.color_name}
            />
          ))}
        </div>

        <div className="mt-4">
          <h3 className="text-lg font-semibold text-(--text-light) mb-2">
            Markers
          </h3>
          <div className="space-y-2">
            {mapData.markers.map((marker, index) => (
              <div
                key={index}
                className="flex items-center gap-3 p-2 rounded bg-(--bg-darker)"
              >
                <div
                  className="w-6 h-6 rounded-full border-2 border-white"
                  style={{ backgroundColor: marker.color }}
                />
                <span className="text-sm text-(--text-light)">
                  {marker.color_name} - ({marker.x.toFixed(1)}%,{" "}
                  {marker.y.toFixed(1)}%)
                </span>
              </div>
            ))}
          </div>
        </div>
      </div>
    </div>
  );
}
