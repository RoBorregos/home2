from config.hri.debug import config as test_hri_config
from config.hri.mocked import config as mocked_hri_config
from subtask_managers.hri_tasks import HRITasks
from subtask_managers.manipulation_tasks import ManipulationTasks
from subtask_managers.nav_tasks import NavigationTasks
from subtask_managers.vision_tasks import VisionTasks

from .task import Task


class SubtaskManager:
    def __init__(self, node, task: Task, mock_areas: list[str] = []):
        self.vision = VisionTasks(node, task=task, mock_data=("vision" in mock_areas))
        self.nav = NavigationTasks(node, task=task, mock_data=("navigation" in mock_areas))
        hri_config = mocked_hri_config if "hri" in mock_areas else test_hri_config
        self.hri = HRITasks(node, config=hri_config, task=task)

        self.manipulation = ManipulationTasks(
            node, task=task, mock_data=("manipulation" in mock_areas)
        )
        self.mocked_areas = mock_areas

    def get_mocked_areas(self):
        return self.mocked_areas
