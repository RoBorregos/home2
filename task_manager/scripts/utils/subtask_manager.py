from subtask_managers.vision_tasks import VisionTasks
from subtask_managers.hri_tasks import HRITasks

# from subtask_managers.manipulation_tasks import ManipulationTasks
from subtask_managers.nav_tasks import NavigationTasks
from config.hri.debug import config as test_hri_config
from .task import Task


class SubtaskManager:
    def __init__(self, node, task: Task, mock_areas: list[str] = []):
        self.vision = VisionTasks(node, task=task, mock_data=("vision" in mock_areas))
        self.hri = HRITasks(node, config=test_hri_config)
        # self.manipulation = ManipulationTasks(node, task=task, mock_data=("manipulation" in mock_areas))
        self.manipulation = None

        self.nav = NavigationTasks(node, mock_data=("navigation" in mock_areas))
