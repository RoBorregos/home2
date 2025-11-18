import ClientTaskDisplay from "./components/ClientTaskDisplay";

export default function Page(props: unknown) {
  const { searchParams } = props as { searchParams?: Record<string, string | string[]> };

  const envTask = process.env.NEXT_PUBLIC_DISPLAY_TASK;

  const taskParam = searchParams?.task;
  const initial =
    (Array.isArray(taskParam) ? taskParam[0] : taskParam) ??
    envTask ??
    "GPSR";

  return <ClientTaskDisplay initialTask={initial} />;
}
