"use client";

import { Suspense } from "react";
import ClientPage from "./client-page";

export default function Page() {
  return (
    <Suspense fallback={<div>Cargando...</div>}>
      <ClientPage />
    </Suspense>
  );
}

