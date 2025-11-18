"use client";

import { useSearchParams } from "next/navigation";
import ClientTaskDisplay from "../app/components/ClientTaskDisplay";

export default function Page() {
  // Next.js hook para leer query params en el cliente
  const searchParams = useSearchParams();

  // Obtenemos la task del query string si existe, sino usamos la env var, sino default a "GPSR"
  const task = searchParams?.get("task") || process.env.NEXT_PUBLIC_DISPLAY_TASK || "GPSR";

  return <ClientTaskDisplay initialTask={task} />;
}

