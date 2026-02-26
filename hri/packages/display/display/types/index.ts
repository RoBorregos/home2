import { z } from "zod";

// Zod schemas
export const markerSchema = z.object({
  x: z.number(),
  y: z.number(),
  color: z.string(),
  color_name: z.string(),
});

export const mapSchema = z.object({
  image_path: z.string(),
  markers: z.array(markerSchema),
});

// Type definitions
export type MapData = z.infer<typeof mapSchema>;

export interface Message {
  type:
    | "heard"
    | "spoken"
    | "keyword"
    | "answer"
    | "user_message"
    | "text_spoken";
  content: string;
  timestamp: Date;
}

export interface AudioState {
<<<<<<< HEAD
  state: "idle" | "listening" | "saying";
=======
  state: "idle" | "listening" | "saying" | "thinking" | "loading";
>>>>>>> 53eaec2f433ebaf3acc49743c2903ceb6f00d99c
  vadLevel: number;
}
