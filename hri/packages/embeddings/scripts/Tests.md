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
```bash
ros2 service call /hri/nlp/embeddings/add_entry_service frida_interfaces/srv/AddEntry "{document: 'rotten_potatoes', collection: 'items'}"
```

```bash
ros2 service call /hri/nlp/embeddings/add_entry_service frida_interfaces/srv/AddEntry "{document: ['rotten_tomatoes', 'apple_pie', 'banana_bread'], collection: 'items'}"

```
#### **Query Entry Service**
```bash
ros2 service call /hri/nlp/embeddings/query_entry_service frida_interfaces/srv/QueryEntry "{query: 'potatoes', topk: 5, collection: 'items'}"
```

#### **Build Embeddings Service**
```bash
ros2 service call /hri/nlp/embeddings/build_embeddings_service frida_interfaces/srv/BuildEmbeddings "{rebuild: true}"
```