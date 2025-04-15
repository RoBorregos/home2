# Testing Guide for Embeddings ROS2 Node


## 1. Manual Testing

### **1.1 Prerequisites**
Before testing, make sure you:
- Have `ros2` installed and sourced.
- Have built your ROS2 workspace (`colcon build`) and sourced it (`source install/setup.bash`).
- Have a running ROS2 environment.

### **1.2 Running the Embeddings Node**
To launch the node:
```bash
ros2 run embeddings item_categorization.py
```
### **1.3 Testing Services Manually**
You can test the services using `ros2 service call`.

#### **Add Entry Service**
It expects document: list of strings, collection: string and optionally metadata: string json
Single entry
```bash
ros2 service call /hri/nlp/embeddings/add_entry_service frida_interfaces/srv/AddEntry "{document: ['rotten_potatoes'], collection: 'items'}"
```
Multiple entry
```bash
ros2 service call /hri/nlp/embeddings/add_entry_service frida_interfaces/srv/AddEntry "{document: ['rotten_tomatoes', 'apple_pie', 'banana_bread'], collection: 'items'}"

```
Entry with metadata
```bash
ros2 service call /hri/nlp/embeddings/add_entry_service frida_interfaces/srv/AddEntry \
"{document: ['apple pie with cinnamon', 'banana_pie', 'mango_pie_with milk'], metadata: '[{\"price\": \"500\"}, {\"price\": \"400\"}, {\"price\": \"450\"}]', collection: 'items'}"
```

#### **Query Entry Service**
It expects query: list of strings, topk: int, collection: string 
Single query
```bash
ros2 service call /hri/nlp/embeddings/query_entry_service frida_interfaces/srv/QueryEntry "{query: ['potatoes'], topk: 5, collection: 'items'}"
```
Multiple query
```bash
ros2 service call /hri/nlp/embeddings/add_entry_service frida_interfaces/srv/AddEntry "{document: ['apple pie with cinnamon', 'banana_pie', 'mango_pie_with milk'], metadata: '{}', collection: 'items'}"
```
#### **Build Embeddings Service**
It expects a bool for rebuilding: eliminate every collection and rebuild(!THIS PROCESS WILL ELIMINATE ALL NEW ADDED ENTRYS)
```bash
ros2 service call /hri/nlp/embeddings/build_embeddings_service frida_interfaces/srv/BuildEmbeddings "{rebuild: true}"
```
### **Runnning automated testing for subtask manager
```bash
ros2 run task_manager test_manager.py 
```
