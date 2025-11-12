import ClientTaskDisplay from "./components/ClientTaskDisplay";

type PageParams = {
  searchParams?: Record<string, string | string[] | undefined> | Promise<Record<string, string | string[] | undefined>>;
};

export default async function Page({ searchParams }: PageParams) {
  const resolved = await Promise.resolve(searchParams);
  const taskParam = resolved?.task;
  const initial = (Array.isArray(taskParam) ? taskParam[0] : taskParam) ?? process.env.NEXT_PUBLIC_DISPLAY_TASK ?? "GPSR";

  return <ClientTaskDisplay initialTask={initial} />;
}