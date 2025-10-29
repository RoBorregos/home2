"use client";

import React from "react";
import { useSearchParams } from "next/navigation";
import TaskDisplay from "./TaskDisplay";

export default function ClientTaskDisplay({ initialTask }: { initialTask: string }) {
  const searchParams = useSearchParams();
  const param = searchParams?.get?.("task") ?? undefined;
  const task = param ?? initialTask ?? "GPSR";

  React.useEffect(() => {
    // quick runtime check in browser console
    // eslint-disable-next-line no-console
    console.debug("ClientTaskDisplay: initialTask=", initialTask, " urlParam=", param, " final=", task);
  }, [initialTask, param, task]);

  return <TaskDisplay task={task} />;
}