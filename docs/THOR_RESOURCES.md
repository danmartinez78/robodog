# Thor GPU Resources and Future Directions

**Last Updated**: 2025-10-10  
**Purpose**: Reference links and notes for Thor optimization and advanced features

---

## 🔗 Key Resources

### Performance Optimization

#### vLLM on Thor (Promising Alternative to Ollama)
- **URL**: https://forums.developer.nvidia.com/t/performance-comparison-of-qwen3-30b-a3b-awq-on-jetson-thor-vs-orin-agx-64gb/345449/5
- **Topic**: Performance comparison of Qwen3-30B-A3B-AWQ on Thor vs Orin AGX 64GB
- **Why Important**: 
  - vLLM may offer better performance than Ollama on Thor
  - Direct performance comparisons between Thor and Orin AGX
  - AWQ quantization techniques for Jetson platforms
  - Community discussion of real-world results
- **Potential Impact**: 
  - Alternative to Ollama if GPU degradation issue persists
  - May solve performance consistency problems
  - Worth investigating if current setup has issues
- **Next Steps**:
  - [ ] Test vLLM on Thor with phi4:14b or qwen models
  - [ ] Compare vLLM vs Ollama performance (speed, stability, memory)
  - [ ] Check if vLLM has model unload/reload degradation issue
  - [ ] Evaluate ease of integration with mission agent

---

#### Jetson AI Stack Documentation
- **URL**: https://elinux.org/Jetson/L4T/Jetson_AI_Stack#AGX_Thor
- **Topic**: Official Jetson AI Stack documentation for Thor
- **Why Important**:
  - Comprehensive guide to AI stack on Thor
  - Official NVIDIA recommendations
  - Integration patterns and best practices
  - Supported frameworks and optimizations
- **Contains**:
  - Installation instructions for AI frameworks
  - Performance tuning guidelines
  - Container configurations
  - Known issues and workarounds
- **Relevant Sections**:
  - AGX Thor specific configurations
  - JetPack 7.x compatibility notes
  - GPU optimization settings
  - Memory management best practices
- **Next Steps**:
  - [ ] Review recommended AI stack configuration
  - [ ] Compare our setup vs official recommendations
  - [ ] Check for Thor-specific optimizations we're missing
  - [ ] Look for CUDA persistence mode settings

---

### Advanced Features and Use Cases

#### Jetson Thor Setup and Demo Guide (PDF)
- **URL**: https://international.download.nvidia.com/JetsonThorReview/Jetson-Thor-Setup-and-Demo-Guide.pdf
- **Topic**: Official Thor setup guide with demo applications
- **Why Important**: 
  - Official NVIDIA documentation
  - Real-world application examples
  - Performance benchmarks and expectations
  - Advanced feature demonstrations

**Key Sections**:

1. **Gr00t (Robot Foundation Model)**
   - Humanoid robot control
   - Foundation model for robotics
   - May be relevant for advanced GO2 behaviors
   - Potential future integration for complex navigation/manipulation

2. **VSS (Video Search and Summarization)** ⭐
   - **Relevance**: Mentioned for separate project
   - Video processing on Thor GPU
   - Real-time video analysis
   - Content summarization capabilities
   - **Use Cases**:
     - Robot camera feed analysis
     - Environment understanding from video
     - Mission logging and review
     - Separate project requirements
   - **Next Steps for VSS Project**:
     - [ ] Review VSS implementation details in PDF
     - [ ] Check hardware requirements (VRAM, compute)
     - [ ] Evaluate integration with Thor setup
     - [ ] Test VSS demo on Thor

3. **Other Demos** (scan PDF for):
   - [ ] LLM inference benchmarks
   - [ ] Vision model examples (relevant for GO2 perception)
   - [ ] Multi-modal AI demonstrations
   - [ ] Performance optimization techniques

---

## 🔍 Investigation Priorities

### Priority 1: Solve Current GPU Degradation Issue
**Before exploring alternatives**:
- [ ] Complete robot testing with phi4:14b + Ollama
- [ ] Install jtop and monitor GPU behavior
- [ ] Test CUDA persistence mode
- [ ] Document reproducible test case

**If degradation persists**:
- [ ] Test vLLM as Ollama alternative (see forum link above)
- [ ] Review Jetson AI Stack docs for optimization hints
- [ ] Contact NVIDIA/Ollama maintainers with findings

---

### Priority 2: vLLM Evaluation (If Needed)

**Why Consider vLLM**:
- Specifically optimized for inference performance
- May handle model loading/unloading better
- Active Jetson community support
- Performance data available (see forum link)

**Evaluation Criteria**:
1. **Performance**: Speed vs Ollama on same models
2. **Stability**: Does model cycling cause degradation?
3. **Memory**: VRAM usage vs Ollama
4. **Integration**: Ease of use with mission agent
5. **Features**: Model support, API compatibility

**Test Plan** (if pursuing):
```bash
# Install vLLM on Thor
pip install vllm

# Test with phi4:14b or qwen models
python -m vllm.entrypoints.openai.api_server \
    --model phi4:14b \
    --host 0.0.0.0 \
    --port 11435

# Benchmark against Ollama
# Use same test prompts
# Compare: speed, memory, stability over time
```

**Integration Notes**:
- vLLM has OpenAI-compatible API
- Mission agent should work with minimal changes
- Update `OLLAMA_BASE_URL` to point to vLLM server
- May need to adjust model names/formats

---

### Priority 3: Advanced Features Exploration

**Gr00t (Robot Foundation Model)**:
- Potential for advanced GO2 behaviors
- Complex navigation and manipulation
- Multi-modal robot control
- Future consideration after basic LLM integration stable

**VSS (Video Search and Summarization)**:
- Read PDF section on VSS implementation
- Evaluate for robot camera feed analysis
- Consider for separate project needs
- May complement GO2 perception skills

**Other Thor Capabilities**:
- Review full PDF for relevant demos
- Check performance benchmarks
- Identify applicable optimizations

---

## 📚 Related Documentation

**In This Repo**:
- `docs/THOR_PERFORMANCE_NOTES.md` - Current GPU issues and workarounds
- `docs/OLLAMA_STATUS_AND_TODOS.md` - Current Ollama setup status
- `scripts/setup_ollama_thor.sh` - Current Ollama container setup
- `scripts/install_jtop_thor.sh` - GPU monitoring installation

**External**:
- NVIDIA Jetson Forums: https://forums.developer.nvidia.com/c/agx-autonomous-machines/jetson-embedded-systems/
- vLLM Documentation: https://docs.vllm.ai/
- Ollama Documentation: https://github.com/ollama/ollama/tree/main/docs

---

## 🎯 Decision Framework

### When to Consider vLLM

**Switch if**:
- ✅ Ollama GPU degradation unsolvable
- ✅ vLLM shows 2x+ performance improvement
- ✅ vLLM is more stable over time
- ✅ Integration effort is reasonable (<1 day)

**Stay with Ollama if**:
- ✅ GPU degradation is solved (CUDA persistence, keep_alive, etc.)
- ✅ Performance is acceptable (>15 tok/s sustained)
- ✅ System is stable with model kept loaded
- ✅ Current integration is working well

### Evaluation Timeline

**Phase 1: Current Setup** (Tomorrow 2025-10-11)
- Test Ollama + phi4:14b on robot
- Monitor with jtop
- Document performance and stability

**Phase 2: Optimization** (If needed, 2025-10-12+)
- Try CUDA optimizations
- Test keep_alive strategies
- Review Jetson AI Stack recommendations

**Phase 3: Alternative Evaluation** (If Phase 2 fails, TBD)
- Set up vLLM test environment
- Benchmark vLLM vs Ollama
- Evaluate integration effort
- Make switch decision

---

## 💡 Notes and Ideas

### vLLM vs Ollama Considerations

**Ollama Pros**:
- ✅ Already set up and working
- ✅ Simple API
- ✅ Good model ecosystem (Modelfile format)
- ✅ Easy model management
- ✅ Mission agent integration complete

**Ollama Cons**:
- ❌ GPU degradation issue (current blocker)
- ❌ May not be optimized for Jetson
- ❌ Limited control over inference settings

**vLLM Pros**:
- ✅ Designed for inference performance
- ✅ Jetson-specific optimizations available
- ✅ Community reports good Thor performance
- ✅ More inference control (batch size, tensor parallel, etc.)

**vLLM Cons**:
- ❌ Not yet tested on our setup
- ❌ May require code changes
- ❌ Different model format (HuggingFace)
- ❌ Additional setup complexity

### VSS Project Notes

**From PDF** (to be filled in after reading):
- Hardware requirements: ___
- Software stack: ___
- Performance expectations: ___
- Integration points: ___

**Potential Applications**:
1. **Robot Camera Analysis**: Real-time video understanding from GO2 cameras
2. **Mission Logging**: Summarize robot missions from video
3. **Environment Mapping**: Video-based scene understanding
4. **Separate Project**: (User mentioned other use case)

**Next Steps**:
- [ ] Read VSS section of PDF thoroughly
- [ ] Document hardware/software requirements
- [ ] Evaluate feasibility on Thor with current setup
- [ ] Consider resource sharing with LLM (memory, GPU)

---

## 🔗 Quick Links Reference

| Resource | URL | Purpose |
|----------|-----|---------|
| vLLM on Thor Forum | [Link](https://forums.developer.nvidia.com/t/performance-comparison-of-qwen3-30b-a3b-awq-on-jetson-thor-vs-orin-agx-64gb/345449/5) | Performance data, alternative to Ollama |
| Jetson AI Stack Docs | [Link](https://elinux.org/Jetson/L4T/Jetson_AI_Stack#AGX_Thor) | Official AI stack guide |
| Thor Setup Guide PDF | [Link](https://international.download.nvidia.com/JetsonThorReview/Jetson-Thor-Setup-and-Demo-Guide.pdf) | Gr00t, VSS, demos |
| NVIDIA Jetson Forums | [Link](https://forums.developer.nvidia.com/c/agx-autonomous-machines/jetson-embedded-systems/) | Community support |
| vLLM Documentation | [Link](https://docs.vllm.ai/) | vLLM setup and usage |
| Ollama Documentation | [Link](https://github.com/ollama/ollama/tree/main/docs) | Current setup reference |

---

## 📝 Action Items

### Immediate (Tomorrow)
- [ ] Test current Ollama setup on robot
- [ ] Monitor with jtop during testing
- [ ] Document any performance issues

### Short-term (This Week)
- [ ] Read Thor Setup PDF (Gr00t and VSS sections)
- [ ] Review Jetson AI Stack documentation
- [ ] Investigate CUDA persistence mode
- [ ] Test model keep_alive strategies

### Medium-term (If Needed)
- [ ] Evaluate vLLM on Thor (forum link)
- [ ] Benchmark vLLM vs Ollama
- [ ] Test VSS demo from PDF
- [ ] Consider integration changes

### Long-term (Future)
- [ ] Explore Gr00t for advanced robot behaviors
- [ ] Evaluate VSS for separate project
- [ ] Optimize Thor AI stack configuration
- [ ] Performance tuning based on production data

---

**Remember**: Current setup may be fine! These are backup options and future directions if needed.

**Priority**: Make current Ollama + phi4:14b work well before exploring alternatives.

---

**Last Updated**: 2025-10-10 23:50  
**Status**: Resources captured, ready for future reference
