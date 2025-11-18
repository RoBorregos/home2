"use client"; // marca todo el archivo como client-side

import ClientTaskDisplay from "./components/ClientTaskDisplay";

// Esto evita que Next intente prerenderizar la página
export const dynamic = "force-dynamic";

export default function Page() {
  return (
    <ClientTaskDisplay
      initialTask={process.env.NEXT_PUBLIC_DISPLAY_TASK || "GPSR"}
    />
  );
}
