"use client";

import { useSearchParams } from "next/navigation";
import ClientTaskDisplay from "./components/ClientTaskDisplay";

export default function ClientPage() {
  const searchParams = useSearchParams();
  const task =
    searchParams?.get("task") ||
    process.env.NEXT_PUBLIC_DISPLAY_TASK ||
    "Storing-groceries";

  return <ClientTaskDisplay initialTask={task} />;
}