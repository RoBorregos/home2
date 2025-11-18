"use client";
import ClientTaskDisplay from "../app/components/ClientTaskDisplay";

export default function Page() {
  const task = process.env.NEXT_PUBLIC_DISPLAY_TASK || "GPSR";
  return <ClientTaskDisplay initialTask={task} />;
}
