# Agent Refactor: DIMOS Alignment Analysis

**Date**: October 5, 2025  
**Status**: ⚠️ **NEEDS REVIEW** - Potential misalignment with DIMOS patterns

---

## 🚨 The Concern

Our agent refactoring may have violated DIMOS conventions by wrapping `OpenAIAgent` and `PlanningAgent` instead of using them directly.

### What We Built

```python
# Our wrapper approach
from shadowhound_mission_agent.agent import AgentFactory

agent = AgentFactory.create(
    agent_type="openai",
    skills=skills,
    robot=robot,
    config={"model": "gpt-4-turbo"}
)

# Returns our OpenAIAgent wrapper class
# which wraps dimos.agents.agent.OpenAIAgent
```

### What DIMOS Expects

```python
# Direct DIMOS usage
from dimos.agents import OpenAIAgent

agent = OpenAIAgent(
    dev_name="ShadowHound",
    tools=robot_skills.get_tools(),  # Auto-generates tool definitions
    model="gpt-4-turbo"
)

# Use directly - no wrapper
response = agent.query("Navigate to kitchen")
```

---

## 🔍 Analysis: Is Our Refactor Wrong?

### DIMOS Architecture (From Documentation)

```
┌────────────────────────────────────────────────────────┐
│              DIMOS Layering                            │
└────────────────────────────────────────────────────────┘

Application Layer
    ├─ Launch files
    └─ Configuration

Agent Layer (DIMOS)
    ├─ OpenAIAgent      ◄── Use directly
    ├─ PlanningAgent    ◄── Use directly
    └─ ClaudeAgent      ◄── Use directly

Skills Layer (DIMOS + Custom)
    ├─ SkillLibrary     ◄── Use directly
    ├─ NavigateToGoal   ◄── DIMOS built-in
    ├─ FollowHuman      ◄── DIMOS built-in
    └─ PatrolArea       ◄── ShadowHound custom

Robot Layer (DIMOS)
    └─ UnitreeGo2       ◄── Use directly
```

### Our Architecture (What We Built)

```
┌────────────────────────────────────────────────────────┐
│              Our Layering                              │
└────────────────────────────────────────────────────────┘

Application Layer
    └─ mission_agent.py (ROS node)

Agent Abstraction Layer ◄── ⚠️ NEW LAYER (is this needed?)
    ├─ BaseAgent (abstract)
    ├─ AgentFactory
    ├─ OpenAIAgent (wrapper)  ◄── Wraps dimos.agents.OpenAIAgent
    └─ PlanningAgent (wrapper) ◄── Wraps dimos.agents.PlanningAgent

DIMOS Layer
    ├─ dimos.agents.OpenAIAgent ◄── Wrapped by us
    └─ dimos.agents.PlanningAgent ◄── Wrapped by us
```

---

## 🤔 Key Questions

### Question 1: Does the abstraction add value?

**Pros of Our Wrapper:**
- ✅ Consistent interface across agent types
- ✅ Structured `MissionResult` with telemetry
- ✅ Factory pattern for agent creation
- ✅ Easier to test without DIMOS
- ✅ Handles Observable pattern from DIMOS
- ✅ Prepares for non-DIMOS agents (local LLMs, VLAs)

**Cons of Our Wrapper:**
- ❌ Extra layer of indirection
- ❌ Hides DIMOS features
- ❌ Deviates from DIMOS patterns
- ❌ More code to maintain
- ❌ Potential to break DIMOS updates

### Question 2: What does DIMOS actually return?

Let's check DIMOS agent return types:

```python
# DIMOS OpenAIAgent
response = agent.query("Go to kitchen")
# Returns: str (the AI's text response)

# DIMOS PlanningAgent  
response = agent.plan_and_execute("Go to kitchen")
# Returns: str (the AI's text response)
```

**DIMOS doesn't return structured results!** It just returns text.

### Question 3: How does DIMOS handle skills?

```python
# DIMOS Pattern
from dimos.agents import OpenAIAgent
from dimos.skills import SkillLibrary

# Get tools from SkillLibrary
robot_skills = robot.get_skills()
tools = robot_skills.get_tools()  # Auto-generates OpenAI function definitions

# Agent uses function calling
agent = OpenAIAgent(
    dev_name="ShadowHound",
    tools=tools,  # ◄── Skills become tools
    model="gpt-4-turbo"
)

# Agent calls skills via function calling
response = agent.query("Go to kitchen")
# Behind the scenes:
# 1. Agent decides to call NavigateToGoal tool
# 2. DIMOS executes the skill
# 3. Agent sees result and continues
# 4. Returns final text response
```

**Key Insight**: DIMOS agents already integrate with SkillLibrary!

---

## 💡 The Problem with Our Design

### We Duplicated Functionality

DIMOS already has:
- ✅ Agent interface (`OpenAIAgent`, `PlanningAgent`)
- ✅ Skills integration (via `tools` parameter)
- ✅ Multiple agent types (OpenAI, Claude, local LLMs)
- ✅ Observable pattern handling

We added:
- 🔄 Another agent interface (`BaseAgent`)
- 🔄 Another factory (`AgentFactory`)
- 🔄 Wrappers that just call DIMOS

### We Lost DIMOS Features

By wrapping, we:
- ❌ Can't access DIMOS semantic memory
- ❌ Can't use DIMOS streaming
- ❌ Can't use DIMOS prompt templates
- ❌ Miss future DIMOS agent updates

---

## 🎯 Two Paths Forward

### Option A: Keep the Wrapper (Justify It)

**When it makes sense:**
- If we need to support **non-DIMOS agents** (local LLMs, custom VLAs)
- If we need **structured results** for telemetry/logging
- If we want to **switch away from DIMOS** in the future

**How to fix:**
```python
# Make wrapper optional
class MissionAgentNode:
    def __init__(self):
        # Option 1: Use DIMOS directly (default)
        if self.config.get("use_dimos_directly", True):
            self.agent = self._init_dimos_agent()
        # Option 2: Use our wrapper (advanced)
        else:
            self.agent = AgentFactory.create(...)
    
    def _init_dimos_agent(self):
        """Use DIMOS agent directly (recommended)."""
        from dimos.agents import OpenAIAgent
        
        # Get skills from robot
        tools = self.robot.get_skills().get_tools()
        
        # Create DIMOS agent
        return OpenAIAgent(
            dev_name="shadowhound",
            tools=tools,
            model="gpt-4-turbo"
        )
```

### Option B: Remove the Wrapper (Align with DIMOS)

**Simpler approach:**
```python
# mission_agent.py - Simplified

from dimos.agents import OpenAIAgent, PlanningAgent

class MissionAgentNode(Node):
    def __init__(self):
        # Initialize robot (DIMOS)
        self.robot = UnitreeGo2(...)
        
        # Get skills (DIMOS)
        self.skills = self.robot.get_skills()
        tools = self.skills.get_tools()
        
        # Create agent (DIMOS directly)
        if self.use_planning:
            self.agent = PlanningAgent(
                robot=self.robot,
                dev_name="shadowhound"
            )
        else:
            self.agent = OpenAIAgent(
                dev_name="shadowhound",
                tools=tools,
                model="gpt-4-turbo"
            )
    
    def mission_callback(self, msg):
        """Handle mission command."""
        command = msg.data
        
        # Execute via DIMOS agent
        if self.use_planning:
            response = self.agent.plan_and_execute(command)
        else:
            response = self.agent.run_observable_query(command).run()
        
        # Publish response
        self.status_pub.publish(String(data=response))
        
        # Broadcast to web
        if self.web:
            self.web.broadcast_sync(f"✅ {response}")
```

**This is exactly what DIMOS examples do!**

---

## 🔄 Recommendation

### Short Term: **Option B (Remove Wrapper)**

1. **Simplify mission_agent.py** to use DIMOS directly
2. **Delete agent module** (base_agent, wrappers, factory)
3. **Follow DIMOS patterns** from their examples
4. **Add custom skills** to SkillLibrary instead

**Rationale:**
- Aligns with DIMOS conventions
- Reduces code complexity
- Easier to update with DIMOS changes
- Leverages DIMOS features we paid for by importing it

### Long Term: **Add Vision Skills (Not Agent Wrapper)**

When we need vision:
```python
# Add vision skills to DIMOS SkillLibrary
from shadowhound_skills.vision import DescribeScene, DetectObjects

# Register with DIMOS
robot_skills = robot.get_skills()
robot_skills.add(DescribeScene)
robot_skills.add(DetectObjects)

# Agent automatically gets these as tools!
tools = robot_skills.get_tools()
agent = OpenAIAgent(dev_name="shadowhound", tools=tools)

# Agent can now call vision skills
response = agent.query("What do you see?")
# Agent calls DescribeScene skill, gets result, responds
```

**This is the DIMOS way!**

---

## ✅ Action Items

### Immediate
1. [ ] **Review this analysis** with team
2. [ ] **Check DIMOS examples** for agent patterns
3. [ ] **Decide**: Keep wrapper or use DIMOS directly?

### If Removing Wrapper
1. [ ] Simplify `mission_agent.py` to use DIMOS directly
2. [ ] Delete `agent/` module
3. [ ] Update tests to test DIMOS integration
4. [ ] Document DIMOS agent usage

### If Keeping Wrapper
1. [ ] Document WHY we're wrapping (specific use cases)
2. [ ] Make wrapper optional (default to DIMOS direct)
3. [ ] Ensure we don't lose DIMOS features
4. [ ] Add tests comparing wrapper vs direct DIMOS

---

## 📚 References

- **DIMOS Agent Examples**: `src/dimos-unitree/tests/test_unitree_agent.py`
- **DIMOS Skills Integration**: `docs/DIMOS_INTEGRATION.md`
- **Skills Pattern**: `docs/DIMOS_CAPABILITIES.md`
- **Our Wrapper**: `src/shadowhound_mission_agent/shadowhound_mission_agent/agent/`

---

## 🤝 Discussion Points

1. **Do we need to swap agents?** Or is DIMOS agent selection enough?
2. **Do we need structured results?** Or is text response sufficient?
3. **Are we planning to use non-DIMOS agents?** (Local LLMs, VLAs)
4. **How important is testability without DIMOS?**

**The answer determines if our wrapper is justified or over-engineering.**
