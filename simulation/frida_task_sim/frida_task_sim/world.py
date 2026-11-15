"""Scenario -> Gazebo world: map walls, venue furniture, objects and an overview camera."""

from frida_gz_sim.world import build_world

from frida_task_sim import catalog, people, venue as venues

WORLD_NAME = "task_sim"

OVERVIEW_CAMERA = """    <model name="overview_camera">
      <static>true</static>
      <pose>{x} {y} {z} 0 1.5708 1.5708</pose>
      <link name="link">
        <sensor name="overview" type="camera">
          <always_on>true</always_on>
          <update_rate>4</update_rate>
          <topic>overview_camera/image</topic>
          <camera>
            <horizontal_fov>1.25</horizontal_fov>
            <image><width>960</width><height>720</height></image>
            <clip><near>0.05</near><far>30</far></clip>
          </camera>
        </sensor>
      </link>
    </model>
"""


def build(scenario: dict) -> str:
    """SDF world string for a scenario, in map coordinates."""
    venue = venues.load(scenario["venue"])
    task = scenario["task"]
    x, y, z = venue[task].get("overview", (0.3, -13.4, 7.0))
    models = OVERVIEW_CAMERA.format(x=x, y=y, z=z)
    if task == "ppc":
        models += venues.door_sdf(venue)
    for i, (sx, sy) in enumerate(scenario.get("seats", [])):
        models += people.seat_sdf(f"seat_{i}", sx, sy)
    for person in scenario.get("people", []):
        models += people.person_sdf(person)
    for name, f in venue["furniture"].items():
        models += venues.furniture_sdf(name, f)
    for obj in scenario["objects"]:
        models += catalog.model_sdf(obj["id"], obj["name"], obj["pose"])
    return build_world(
        venues.map_yaml(venue),
        WORLD_NAME,
        wall_height=1.2,
        collisions=False,
        extra_models=models,
        keep_box=venues.keep_wall(venue),
    )
