import ClientTaskDisplay from "./components/ClientTaskDisplay";

type Props = {
  searchParams?: { task?: string };
};

export default function Page({ searchParams }: Props) {
  const envTask = process.env.NEXT_PUBLIC_DISPLAY_TASK;
  const initial = searchParams?.task ?? envTask ?? "GPSR";

  return <ClientTaskDisplay initialTask={initial} />;
}