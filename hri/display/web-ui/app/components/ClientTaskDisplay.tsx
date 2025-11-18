"use client";

import React from "react";
import { useSearchParams } from "next/navigation";
import TaskDisplay from "./TaskDisplay";

type Props = {
  initialTask: string;
};

export default function ClientTaskDisplay({ initialTask }: Props) {
  const searchParams = useSearchParams();
  const param = searchParams?.get?.("task") ?? undefined;
  const task = param ?? initialTask ?? "GPSR";

  React.useEffect(() => {
    console.debug(
      "ClientTaskDisplay: initialTask=",
      initialTask,
      " urlParam=",
      param,
      " final=",
      task
    );
  }, [initialTask, param, task]);

  return <TaskDisplay task={task} />;
}
