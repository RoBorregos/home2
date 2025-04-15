import pytest
from unittest.mock import MagicMock
from task_manager.scripts.subtask_managers.hri_tasks import (
    HRITasks,
)


@pytest.fixture
def mock_node():
    mock = MagicMock()
    mock.get_clock.return_value.now.return_value.nanoseconds = 0
    return mock


@pytest.fixture
def hri_tasks(mock_node):
    hri = HRITasks(task_manager=mock_node)
    return hri


def test_add_item_success(hri_tasks):
    fake_future = MagicMock()
    fake_future.result.return_value.success = True

    hri_tasks.add_item_client.call_async.return_value = fake_future

    result = hri_tasks.add_item(["banana"], [{"context": "fruit"}])
    assert result == "Success"


def test_add_item_failure(hri_tasks):
    fake_future = MagicMock()
    fake_future.result.return_value.success = False
    fake_future.result.return_value.message = "Error message"

    hri_tasks.add_item_client.call_async.return_value = fake_future

    result = hri_tasks.add_item(["banana"], [{"context": "fruit"}])
    assert result == "Failed: Error message"


def test_query_item(hri_tasks):
    fake_future = MagicMock()
    fake_future.result.return_value.results = ["banana"]

    hri_tasks.query_item_client.call_async.return_value = fake_future

    result = hri_tasks.query_item("fruit", top_k=1)
    assert result == ["banana"]


def test_query_location(hri_tasks):
    fake_future = MagicMock()
    fake_future.result.return_value.results = ["kitchen"]

    hri_tasks.query_item_client.call_async.return_value = fake_future

    result = hri_tasks.query_location("room", top_k=1)
    assert result == ["kitchen"]
