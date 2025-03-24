"use client";
export default function Home() {
  const call = async () => {
    await fetch("/api/ros");
  };

  return (
    <main className="flex flex-col items-center justify-center min-h-screen py-2">
      <h1 className="text-3xl">FRIDA</h1>
      <button onClick={() => call()}>Start ROS Node</button>
    </main>
  );
}
