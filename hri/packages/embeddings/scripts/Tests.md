# Testing Guide for Embeddings ROS2 Node


## 1. Manual Testing

### **1.1 Prerequisites**
Before testing, make sure you:
- Have `ros2` installed and sourced.
- Have built your ROS2 workspace (`colcon build`) and sourced it (`source install/setup.bash`).
- Have a running ROS2 environment.
    ros2 service call /hri/nlp/embeddings/add_entry_service frida_interfaces/srv/AddEntry "{document: 'rotten_tomatoes', collection: 'items', metadata: '[{\"context\": \"not in the menu\"}]'}"
### **1.2 Running the Embeddings Node**
To launch the node:
```bash
ros2 run embeddings item_categorization.py
```

### **1.3 Testing Services Manually**
You can test the services using `ros2 service call`.

#### **Add Item Service**
```bash
ros2 service call /hri/nlp/embeddings/add_item_service frida_interfaces/srv/AddItem "{collection: 'test_collection', document: 'test_item', metadata: '{\"category\": \"electronics\"}'}"
```

#### **Query Item Service**
```bash
ros2 service call /hri/nlp/embeddings/query_item_service frida_interfaces/srv/QueryItem "{query: 'test_item', topk: 5, return_location: false}"
```

#### **Add Location Service**
```bash
ros2 service call /hri/nlp/embeddings/add_location_service frida_interfaces/srv/AddLocation "{collection: 'locations', document: 'kitchen'}"
```

#### **Query Location Service**
```bash
ros2 service call /hri/nlp/embeddings/query_location_service frida_interfaces/srv/QueryLocation "{query: 'kitchen', topk: 3, return_coord: false}"
```

#### **Build Embeddings Service**
```bash
ros2 service call /hri/nlp/embeddings/build_embeddings_service frida_interfaces/srv/BuildEmbeddings "{rebuild: true}"
```

---
## 2. Automated Unit Testing
To automate testing, we use `pytest`.

### **2.1 Installing Dependencies**
Ensure `pytest` is installed:
```bash
pip install pytest
```

### **2.2 Running Unit Tests**
Navigate to your package directory and run:
```bash
pytest tests/
```

### **2.3 Sample Unit Test (tests/test_embeddings.py)**
Create a `tests/` directory inside your package and add a test script:

```python
import pytest
from your_embeddings_script import Embeddings
from frida_interfaces.srv import AddItem

@pytest.fixture
def embeddings_node():
    return Embeddings()

def test_add_item(embeddings_node):
    request = AddItem.Request()
    request.collection = "test_collection"
    request.document = "item_1"
    request.metadata = '{"category": "electronics"}'

    response = embeddings_node.add_item_callback(request, AddItem.Response())

    assert response.success, "Item was not added successfully"
    assert response.message == "Item added successfully"
```

---
## 3. Debugging and Logs
If tests fail, check logs with:
```bash
ros2 topic echo /rosout
```
Or check errors in the terminal output.

---
## 4. Conclusion
- **Manual testing** ensures basic functionality.
- **Automated tests** help maintain code quality.
- Run **unit tests before deploying** to catch errors early.

If you encounter issues, report them to the development team.

