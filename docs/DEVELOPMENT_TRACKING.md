# Development Tracking Guide

Quick reference for using the ShadowHound development tracking system.

---

## Files

### 📖 DEVLOG.md
**Purpose**: Chronological record of what was done and why

**When to update**:
- After completing a major feature
- After fixing significant bugs
- After making important architectural decisions
- End of each development session (optional)

**What to include**:
- What was done (brief summary)
- Technical details (implementation specifics)
- Validation (how it was tested)
- Key learnings (insights, gotchas, best practices)
- Next steps (follow-up work)

**Template**: See bottom of DEVLOG.md for entry template

---

### ✅ TODO.md
**Purpose**: Organized list of work to be done

**When to update**:
- When starting new work (move from todo → in progress)
- When completing work (move to "Recently Completed")
- When discovering new tasks
- During planning sessions

**Priority levels**:
- 🔴 **High**: Critical, blocking, or time-sensitive
- 🟡 **Medium**: Important but not urgent
- 🟢 **Low**: Nice to have, can wait

**Task format**:
```markdown
- [ ] Clear task title
  - Context: Why it's needed
  - Acceptance: How to verify completion
  - Related: Links to files/docs
```

---

## Workflows

### Starting a New Task

1. **Check TODO.md** for next priority item
2. **Move to in-progress** (add 🔄 emoji or note)
3. **Work on the task**
4. **Update as you go** if you discover subtasks

### Completing a Task

1. **Test and validate** the work
2. **Update TODO.md**:
   - Check off the item [x]
   - Move to "Recently Completed" section
   - Add completion date
3. **Update DEVLOG.md**:
   - Add entry with what was done
   - Include technical details and learnings
4. **Commit both files** with descriptive message

### End of Day

1. **Review TODO.md**: Update any in-progress items
2. **Optional DEVLOG.md entry**: Summary of the day
3. **Commit tracking files**: Keep them current

### Sprint Planning

1. **Review TODO.md**: Re-prioritize tasks
2. **Add new tasks** discovered since last planning
3. **Estimate effort** for high-priority items
4. **Set milestone goals**

---

## Tips

### DEVLOG Best Practices

✅ **DO**:
- Write while context is fresh
- Include code snippets for complex changes
- Document "why" not just "what"
- Record failed approaches (save others time)
- Link to related files/commits

❌ **DON'T**:
- Wait until end of week (you'll forget details)
- Just list commits (add context and reasoning)
- Skip learnings section (most valuable part)
- Write novels (keep it concise)

### TODO Best Practices

✅ **DO**:
- Keep descriptions clear and actionable
- Add acceptance criteria
- Link to related docs/files
- Review and prune regularly
- Celebrate completed items

❌ **DON'T**:
- Create vague tasks ("improve system")
- Let completed tasks pile up
- Forget to reprioritize
- Add tasks without context
- Leave tasks in limbo (complete or cancel)

---

## Git Integration

### Commit Message Template
```
<type>: <subject>

- Related TODO: [Task name from TODO.md]
- Updates: DEVLOG.md with entry for <date>

<body with additional details>
```

### Suggested Workflow
```bash
# After completing work
git add <changed files>
git add DEVLOG.md TODO.md
git commit -m "feat: Add awesome feature

- Related TODO: Add awesome feature (now completed)
- Updates: DEVLOG.md with 2025-10-06 entry"
```

---

## Quick Commands

### View recent devlog entries
```bash
head -n 50 DEVLOG.md
```

### View high priority tasks
```bash
grep -A 3 "## 🔴 High Priority" TODO.md
```

### View recently completed
```bash
grep -A 10 "## ✅ Recently Completed" TODO.md
```

### Search devlog for topic
```bash
grep -i "keyword" DEVLOG.md
```

### Count pending tasks
```bash
grep -c "^- \[ \]" TODO.md
```

---

## Integration with Development Flow

### Morning Routine
1. `cat TODO.md | head -n 30` - Review priorities
2. Pick next task
3. Start working

### After Completing Task
1. Test and validate
2. Update TODO.md (check off, move to completed)
3. Add DEVLOG.md entry
4. Commit code + tracking files
5. Take a break! ☕

### Before Pushing
1. Review DEVLOG.md - Does it tell the story?
2. Review TODO.md - Is it current?
3. Commit any updates
4. Push to remote

---

## Example Entry

### DEVLOG.md Entry
```markdown
## 2025-10-06 - Added Telemetry Dashboard

### What Was Done
Implemented real-time telemetry dashboard showing mission status,
skill execution metrics, and robot health.

### Technical Details
- Used WebSocket for real-time updates
- Dashboard at http://localhost:8080/telemetry
- Stores last 1000 events in memory

### Validation
- Tested with 3 concurrent missions
- Verified metrics accuracy
- Load tested with 100 events/sec

### Key Learnings
- WebSockets more efficient than polling for real-time data
- Need to limit event buffer size to prevent memory growth
- Plotly.js great for live updating charts

### Next Steps
- Add historical data storage
- Implement alerts for anomalies
```

### TODO.md Update
```markdown
- [x] Add telemetry dashboard (completed 2025-10-06)
  - Real-time mission status
  - Performance metrics
  - Historical data visualization
```

---

**Remember**: The goal is to maintain momentum and capture knowledge, not create busywork. Keep it simple, keep it current, keep it useful.
