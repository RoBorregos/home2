"""
Class to define subtask managers
"""

from subtask_managers.vision_tasks import VisionTasks
from subtask_managers.hri_tasks import HRITasks
from subtask_managers.new_subtask import ManipulationTasks


class SubtaskManager:
    vision: VisionTasks
    hri: HRITasks
    manipulation: ManipulationTasks

    def __init__(self, node, task: str = "DEMO", mock_data: bool = False) -> None:
        self.vision = VisionTasks(node, task=task, mock_data=False)
        # self.hri = HRITasks(node, config=test_hri_config)
        self.manipulation = ManipulationTasks(node, task=task, mock_data=False)
