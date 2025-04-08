export default function Page() {
  return (
    <div>
      <img
        src="http://localhost:8080/stream?topic=/zed/zed_node/right/image_rect_color"
        alt="ROS Image"
        width={640}
        height={480}
        style={{ border: "1px solid black", margin: "10px" }}
      />
    </div>
  );
}
