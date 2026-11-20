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
  state: "idle" | "listening" | "saying" | "thinking" | "loading";
  vadLevel: number;
}
