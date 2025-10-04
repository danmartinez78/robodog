# RAG (Retrieval-Augmented Generation) Integration Guide

## Overview

**Yes!** DIMOS agents have built-in RAG support using **ChromaDB** vector databases. You can supply your own knowledge base, and the agent will automatically retrieve relevant context before querying ChatGPT.

## How RAG Works in ShadowHound

```
┌────────────────────────────────────────────────────────────┐
│ 1. USER COMMAND                                            │
│    "Tell me about the last patrol"                         │
└──────────────────┬─────────────────────────────────────────┘
                   ▼
┌────────────────────────────────────────────────────────────┐
│ 2. AGENT QUERIES VECTOR DB                                 │
│    - Embeds query: "last patrol"                           │
│    - Searches ChromaDB for similar documents               │
│    - Retrieves top 4 most relevant results                 │
│    - Example results:                                      │
│      • "Patrol completed at 14:30, route A"               │
│      • "Detected anomaly in sector 3"                      │
│      • "Battery level: 45% after patrol"                   │
└──────────────────┬─────────────────────────────────────────┘
                   ▼
┌────────────────────────────────────────────────────────────┐
│ 3. BUILDS ENHANCED PROMPT                                  │
│    System: "You control a Unitree Go2..."                  │
│    Context: [Retrieved documents from vector DB]           │
│    User: "Tell me about the last patrol"                   │
└──────────────────┬─────────────────────────────────────────┘
                   ▼
┌────────────────────────────────────────────────────────────┐
│ 4. SENDS TO CHATGPT                                        │
│    ChatGPT now has relevant context from your data!        │
└──────────────────┬─────────────────────────────────────────┘
                   ▼
┌────────────────────────────────────────────────────────────┐
│ 5. RETURNS INFORMED RESPONSE                               │
│    "The last patrol on route A completed at 14:30.         │
│     An anomaly was detected in sector 3. Battery is 45%."  │
└────────────────────────────────────────────────────────────┘
```

## Current Implementation

By default, agents use **OpenAISemanticMemory** with minimal context. Here's what's in your agent now:

```python
# In mission_agent.py (line ~120)
self.agent = OpenAIAgent(
    robot=self.robot,
    dev_name="shadowhound",
    agent_type="Mission",
    skills=self.skills,
    # agent_memory=None  ← Uses default OpenAISemanticMemory
)
```

The default memory has only basic context:
- Optical Flow explanation
- Edge Detection explanation  
- Video/Colors/JSON basics

## Customizing RAG Memory

### Option 1: Use Custom OpenAI Embeddings

Create your own semantic memory with your knowledge:

```python
# In mission_agent.py
from dimos.agents.memory.chroma_impl import OpenAISemanticMemory

def _init_agent(self):
    # Create custom semantic memory
    custom_memory = OpenAISemanticMemory(
        collection_name="shadowhound_knowledge",
        model="text-embedding-3-large",  # OpenAI embedding model
        dimensions=1024
    )
    
    # Add your knowledge base
    custom_memory.add_vector(
        "patrol_history_001",
        "Last patrol completed on 2025-10-03 at 14:30. Route A. Battery: 45%."
    )
    custom_memory.add_vector(
        "anomaly_log_001",
        "Anomaly detected in sector 3 during patrol. Thermal signature elevated."
    )
    custom_memory.add_vector(
        "robot_capabilities",
        "Unitree Go2 can navigate rough terrain, climb stairs, and operate for 2 hours."
    )
    custom_memory.add_vector(
        "safety_protocols",
        "Always check battery before missions. Return to base if battery < 20%."
    )
    
    # Create agent with custom memory
    self.agent = OpenAIAgent(
        robot=self.robot,
        dev_name="shadowhound",
        agent_type="Mission",
        skills=self.skills,
        agent_memory=custom_memory,  # ← Your custom RAG database
        rag_query_n=4,                # Number of results to retrieve
        rag_similarity_threshold=0.45  # Minimum similarity score
    )
```

### Option 2: Use Local Embeddings (No OpenAI Cost!)

Use local sentence transformers instead of OpenAI embeddings:

```python
from dimos.agents.memory.chroma_impl import LocalSemanticMemory

def _init_agent(self):
    # Create local semantic memory (free!)
    local_memory = LocalSemanticMemory(
        collection_name="shadowhound_local",
        model_name="sentence-transformers/all-MiniLM-L6-v2"  # Local model
    )
    
    # Add knowledge base
    local_memory.add_vector("doc_001", "Your knowledge here...")
    
    # Create agent with local memory
    self.agent = OpenAIAgent(
        robot=self.robot,
        dev_name="shadowhound",
        agent_type="Mission",
        skills=self.skills,
        agent_memory=local_memory,  # ← Local RAG database
    )
```

### Option 3: Load Knowledge from Files

Load your knowledge base from text files, CSVs, or databases:

```python
import os
import json

def _init_agent(self):
    from dimos.agents.memory.chroma_impl import OpenAISemanticMemory
    
    # Create memory
    memory = OpenAISemanticMemory(collection_name="shadowhound_docs")
    
    # Load from text files
    docs_dir = "/path/to/knowledge_base"
    for filename in os.listdir(docs_dir):
        if filename.endswith(".txt"):
            with open(os.path.join(docs_dir, filename)) as f:
                content = f.read()
                doc_id = filename.replace(".txt", "")
                memory.add_vector(doc_id, content)
    
    # Or load from JSON
    with open("patrol_logs.json") as f:
        logs = json.load(f)
        for log in logs:
            memory.add_vector(
                log["id"],
                f"Patrol {log['id']}: {log['route']} at {log['time']}. "
                f"Status: {log['status']}. Notes: {log['notes']}"
            )
    
    # Or load from database
    # import sqlite3
    # conn = sqlite3.connect("robot_logs.db")
    # cursor = conn.execute("SELECT id, content FROM knowledge")
    # for row in cursor:
    #     memory.add_vector(row[0], row[1])
    
    self.agent = OpenAIAgent(
        robot=self.robot,
        skills=self.skills,
        agent_memory=memory
    )
```

### Option 4: Persistent ChromaDB

By default, ChromaDB is in-memory. For persistence across restarts:

```python
import chromadb
from langchain_openai import OpenAIEmbeddings
from langchain_chroma import Chroma

def _init_agent(self):
    # Create persistent ChromaDB client
    persist_directory = "/workspaces/shadowhound/data/chroma_db"
    os.makedirs(persist_directory, exist_ok=True)
    
    # Create embeddings
    embeddings = OpenAIEmbeddings(
        model="text-embedding-3-large",
        api_key=os.getenv("OPENAI_API_KEY")
    )
    
    # Create persistent vector store
    vectorstore = Chroma(
        collection_name="shadowhound_persistent",
        embedding_function=embeddings,
        persist_directory=persist_directory  # ← Saves to disk!
    )
    
    # Add documents (only once, they persist!)
    if vectorstore._collection.count() == 0:  # Check if empty
        self.get_logger().info("Initializing knowledge base...")
        vectorstore.add_texts(
            ids=["doc1", "doc2", "doc3"],
            texts=[
                "Robot capabilities and specifications...",
                "Safety protocols and procedures...",
                "Mission history and logs..."
            ]
        )
    
    # Create custom memory wrapper
    class PersistentMemory:
        def __init__(self, vectorstore):
            self.db_connection = vectorstore
        
        def query(self, query_texts, n_results=4, similarity_threshold=None):
            if similarity_threshold:
                return self.db_connection.similarity_search_with_relevance_scores(
                    query=query_texts,
                    k=n_results,
                    score_threshold=similarity_threshold
                )
            else:
                docs = self.db_connection.similarity_search(query_texts, k=n_results)
                return [(doc, None) for doc in docs]
        
        def add_vector(self, vector_id, vector_data):
            self.db_connection.add_texts(
                ids=[vector_id],
                texts=[vector_data]
            )
    
    memory = PersistentMemory(vectorstore)
    
    self.agent = OpenAIAgent(
        robot=self.robot,
        skills=self.skills,
        agent_memory=memory
    )
```

## RAG Configuration Parameters

```python
self.agent = OpenAIAgent(
    # ... other params ...
    agent_memory=custom_memory,           # Your vector database
    rag_query_n=4,                        # Number of documents to retrieve
    rag_similarity_threshold=0.45,        # Minimum similarity (0-1)
                                          # Higher = more strict matching
)
```

### Tuning RAG Parameters

**`rag_query_n`** (default: 4):
- Number of most relevant documents to retrieve
- Higher = more context, but uses more tokens
- Typical: 3-5 for focused context, 10+ for broad context

**`rag_similarity_threshold`** (default: 0.45):
- Minimum similarity score (0.0 = no match, 1.0 = perfect match)
- Higher = only very relevant docs, fewer false positives
- Lower = more docs, but may include less relevant info
- Typical: 0.4-0.6 for general use

## Complete Example: Mission History RAG

Here's a complete example adding mission history to your agent:

```python
# mission_agent.py

from dimos.agents.memory.chroma_impl import OpenAISemanticMemory
from datetime import datetime

def _init_agent(self):
    """Initialize the DIMOS agent with custom RAG memory."""
    
    # Create semantic memory for mission history
    mission_memory = OpenAISemanticMemory(
        collection_name="shadowhound_missions",
        model="text-embedding-3-large",
        dimensions=1024
    )
    
    # Add mission knowledge base
    knowledge_base = [
        {
            "id": "mission_001",
            "content": "Patrol mission on 2025-10-01. Route: Perimeter A. "
                      "Duration: 45 minutes. Battery start: 100%, end: 65%. "
                      "No anomalies detected. Weather: Clear, 22°C."
        },
        {
            "id": "mission_002",
            "content": "Inspection mission on 2025-10-02. Target: Building 3. "
                      "Found thermal anomaly in sector B. Reported to operator. "
                      "Mission successful. Battery: 78% → 55%."
        },
        {
            "id": "capability_movement",
            "content": "Unitree Go2 movement capabilities: Walk speed 0-1.6 m/s, "
                      "trot speed 0-3.5 m/s, can climb 45° slopes, 20cm obstacles, "
                      "stairs up to 18cm step height. IP67 waterproof rating."
        },
        {
            "id": "capability_sensors",
            "content": "Unitree Go2 sensors: 5x RGB cameras, 1x lidar, IMU, "
                      "proprioceptive joint sensors. Can detect obstacles up to 10m. "
                      "Thermal camera optional. GPS accuracy ±2m."
        },
        {
            "id": "safety_battery",
            "content": "Safety protocol: Battery management. Minimum 20% required "
                      "for mission start. Return to base automatically at 15%. "
                      "Low battery warning at 25%. Full charge takes 90 minutes."
        },
        {
            "id": "safety_terrain",
            "content": "Safety protocol: Terrain navigation. Avoid water deeper "
                      "than 3cm, slopes steeper than 45°, gaps wider than 30cm. "
                      "Use slow mode in crowded areas. Maximum safe speed: 1.5 m/s indoors."
        }
    ]
    
    # Populate memory
    self.get_logger().info("Loading knowledge base into RAG memory...")
    for item in knowledge_base:
        mission_memory.add_vector(item["id"], item["content"])
    self.get_logger().info(f"Loaded {len(knowledge_base)} documents into memory")
    
    # Create agent with RAG-enabled memory
    try:
        self.agent = OpenAIAgent(
            robot=self.robot,
            dev_name="shadowhound",
            agent_type="Mission",
            skills=self.skills,
            agent_memory=mission_memory,   # ← Your RAG database
            rag_query_n=3,                 # Retrieve top 3 docs
            rag_similarity_threshold=0.5,  # Moderate strictness
            model_name="gpt-4o",
        )
        self.get_logger().info("Agent initialized with RAG memory")
        
    except Exception as e:
        self.get_logger().error(f"Failed to initialize agent: {e}")
        raise
```

Now when users ask questions, the agent has context:

```bash
# Web dashboard or ROS topic
"What was the last mission?"
# RAG retrieves: mission_002 content
# ChatGPT response: "The last mission was an inspection of Building 3 on 
# October 2nd, where a thermal anomaly was found in sector B..."

"Can the robot climb stairs?"
# RAG retrieves: capability_movement content
# ChatGPT response: "Yes, the Unitree Go2 can climb stairs with steps up 
# to 18cm high. It can also handle 45° slopes and 20cm obstacles..."

"What's the battery protocol?"
# RAG retrieves: safety_battery content
# ChatGPT response: "Battery safety requires minimum 20% to start missions. 
# The robot will automatically return at 15%. Low battery warning at 25%..."
```

## Dynamic RAG: Adding Data at Runtime

You can add new knowledge during runtime:

```python
# In your mission callback or web handler
def mission_callback(self, msg: String):
    command = msg.data
    
    # Execute mission
    result = self.agent.process_text(command)
    
    # Add result to RAG memory for future reference
    mission_id = f"mission_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
    mission_log = f"Mission executed at {datetime.now()}: {command}. Result: {result}"
    
    if hasattr(self.agent, 'agent_memory') and self.agent.agent_memory:
        self.agent.agent_memory.add_vector(mission_id, mission_log)
        self.get_logger().info(f"Added mission to memory: {mission_id}")
```

Now the robot learns from every mission!

## Testing RAG

Test that RAG is working:

```python
# After agent initialization
def test_rag(self):
    """Test RAG retrieval."""
    if not self.agent.agent_memory:
        self.get_logger().warn("No agent memory configured")
        return
    
    # Query the vector database
    test_query = "battery safety"
    results = self.agent.agent_memory.query(
        test_query,
        n_results=3,
        similarity_threshold=0.3
    )
    
    self.get_logger().info(f"RAG test query: '{test_query}'")
    for i, (doc, score) in enumerate(results):
        self.get_logger().info(f"Result {i+1}:")
        self.get_logger().info(f"  Score: {score}")
        self.get_logger().info(f"  Content: {doc.page_content[:100]}...")
```

## Best Practices

### 1. Document Chunking
For large documents, split into chunks:

```python
def chunk_document(text, chunk_size=500, overlap=50):
    """Split document into overlapping chunks."""
    chunks = []
    for i in range(0, len(text), chunk_size - overlap):
        chunk = text[i:i + chunk_size]
        chunks.append(chunk)
    return chunks

# Use it
large_doc = "Very long document content..."
chunks = chunk_document(large_doc)
for i, chunk in enumerate(chunks):
    memory.add_vector(f"doc_001_chunk_{i}", chunk)
```

### 2. Metadata Tagging
Add metadata for filtering:

```python
# Using raw ChromaDB with metadata
vectorstore.add_texts(
    ids=["mission_001"],
    texts=["Mission content..."],
    metadatas=[{
        "type": "mission",
        "date": "2025-10-01",
        "status": "completed",
        "battery": 65
    }]
)

# Query with metadata filter (advanced ChromaDB usage)
results = vectorstore.similarity_search(
    "patrol missions",
    filter={"type": "mission", "status": "completed"}
)
```

### 3. Periodic Cleanup
Remove old/irrelevant documents:

```python
# Delete old missions
old_mission_ids = ["mission_001", "mission_002"]
for doc_id in old_mission_ids:
    self.agent.agent_memory.delete_vector(doc_id)
```

### 4. Cost Optimization
Local embeddings save money:

```python
# OpenAI embeddings: ~$0.00013 per 1K tokens
# Local embeddings: FREE!

# Use local for development, OpenAI for production
if os.getenv("ENVIRONMENT") == "development":
    memory = LocalSemanticMemory(...)  # Free!
else:
    memory = OpenAISemanticMemory(...)  # Better quality
```

## Troubleshooting

### RAG Not Working

**Check 1**: Verify memory is initialized
```python
if self.agent.agent_memory:
    print("Memory initialized ✓")
else:
    print("Memory NOT initialized ✗")
```

**Check 2**: Test direct query
```python
results = self.agent.agent_memory.query("test", n_results=1)
print(f"Found {len(results)} results")
```

**Check 3**: Check similarity threshold
```python
# Too high threshold = no results
rag_similarity_threshold=0.9  # ✗ Too strict

# Lower threshold
rag_similarity_threshold=0.3  # ✓ More permissive
```

### ChromaDB Errors

```bash
# Install dependencies
pip install chromadb langchain-openai langchain-chroma sentence-transformers
```

### Embedding Errors

```python
# Ensure OPENAI_API_KEY is set for OpenAISemanticMemory
import os
if not os.getenv("OPENAI_API_KEY"):
    raise Exception("Set OPENAI_API_KEY for OpenAI embeddings")

# Or use local embeddings (no API key needed)
memory = LocalSemanticMemory(...)  # No API key required
```

## Advanced: Custom Memory Implementation

Create your own memory backend:

```python
from dimos.agents.memory.base import AbstractAgentSemanticMemory

class CustomMemory(AbstractAgentSemanticMemory):
    """Your custom vector database integration."""
    
    def __init__(self):
        # Your database connection
        super().__init__(connection_type='local')
    
    def create(self):
        # Initialize your database
        pass
    
    def add_vector(self, vector_id, vector_data):
        # Add to your database
        pass
    
    def query(self, query_texts, n_results=4, similarity_threshold=None):
        # Query your database
        # Return: List[Tuple[Document, Optional[float]]]
        pass
    
    # Implement other abstract methods...
```

## Summary

**✅ Yes, you can supply a RAG vector database!**

**Steps**:
1. Create `OpenAISemanticMemory` or `LocalSemanticMemory`
2. Add your knowledge: `memory.add_vector(id, content)`
3. Pass to agent: `agent_memory=memory`
4. Configure: `rag_query_n` and `rag_similarity_threshold`

**Benefits**:
- Agent has context about your robot's history
- Answers questions based on your data
- No prompt engineering needed
- Automatic retrieval before ChatGPT calls

**Costs**:
- OpenAI embeddings: ~$0.00013/1K tokens
- Local embeddings: FREE!
- ChatGPT calls: Same as before (but with better context)

**Next Steps**:
1. Decide: OpenAI or local embeddings?
2. Prepare your knowledge base (text files, logs, docs)
3. Update `_init_agent()` in `mission_agent.py`
4. Test with queries that need context

Your RAG-enhanced agent will be much smarter about your specific robot and missions! 🚀
