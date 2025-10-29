import TaskDisplay from "./components/TaskDisplay";

type Props = {
  searchParams?: { task?: string };
};

export default function Page({ searchParams }: Props) {
  const envTask = process.env.NEXT_PUBLIC_DISPLAY_TASK;
  const task = searchParams?.task ?? envTask ?? "GPSR";

  return <TaskDisplay task={task} />;
}