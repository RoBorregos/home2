<<<<<<< HEAD
from task_manager.config.hri.debug import config as test_hri_config
from task_manager.config.hri.mocked import config as mocked_hri_config
=======
>>>>>>> 297bdd4b3af41d15ae85514039ca5b3f1ed42c3a
from task_manager.subtask_managers.hri_tasks import HRITasks
from task_manager.subtask_managers.manipulation_tasks import ManipulationTasks
from task_manager.subtask_managers.nav_tasks import NavigationTasks
from task_manager.subtask_managers.vision_tasks import VisionTasks
<<<<<<< HEAD

=======
>>>>>>> 297bdd4b3af41d15ae85514039ca5b3f1ed42c3a
from .task import Task


class SubtaskManager:
    def __init__(self, node, task: Task, mock_areas: list[str] = []):
        self.vision = VisionTasks(node, task=task, mock_data=("vision" in mock_areas))
        self.nav = NavigationTasks(node, task=task, mock_data=("navigation" in mock_areas))
        self.manipulation = ManipulationTasks(
            node, task=task, mock_data=("manipulation" in mock_areas)
        )
        self.hri = HRITasks(node, task=task, mock_data=("hri" in mock_areas))

        self.mocked_areas = mock_areas

    def get_mocked_areas(self):
        return self.mocked_areas
