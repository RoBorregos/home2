from task_manager.subtask_managers.hri_tasks import HRITasks
from task_manager.subtask_managers.manipulation_tasks import ManipulationTasks
from task_manager.subtask_managers.nav_tasks import NavigationTasks
from task_manager.subtask_managers.vision_tasks import VisionTasks
from .task import Task


class SubtaskManager:
    def __init__(self, node, task: Task, mock_areas: list[str] = []):
        self.vision = VisionTasks(node, task=task, mock_data=("vision" in mock_areas))
        self.nav = NavigationTasks(node, task=task, mock_data=("navigation" in mock_areas))
        self.manipulation = ManipulationTasks(node, task=task, mock_data=("manipulation" in mock_areas))
        self.hri = HRITasks(node, task=task, mock_data=("hri" in mock_areas))

        self.mocked_areas = mock_areas

    def get_mocked_areas(self):
        return self.mocked_areas
