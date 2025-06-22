import React, { useRef, useState, useEffect } from "react";

const CELL_SIZE = 40;
const ROWS = 10;
const COLS = 10;
const CANVAS_WIDTH = CELL_SIZE * COLS;
const CANVAS_HEIGHT = CELL_SIZE * ROWS;

const AGENT_COLORS = ["red", "blue", "green", "orange", "purple"];

function App() {
  const canvasRef = useRef(null);
  const [agents, setAgents] = useState([]);
  const [conflictMap, setConflictMap] = useState({});
  const [currentTime, setCurrentTime] = useState(0);
  const [isRunning, setIsRunning] = useState(false);

  // Draw grid
  const drawGrid = (ctx) => {
    ctx.clearRect(0, 0, CANVAS_WIDTH, CANVAS_HEIGHT);
    ctx.strokeStyle = "#aaa";
    for (let r = 0; r < ROWS; r++) {
      for (let c = 0; c < COLS; c++) {
        ctx.strokeRect(c * CELL_SIZE, r * CELL_SIZE, CELL_SIZE, CELL_SIZE);
      }
    }
  };

  // Draw conflicts (red transparent overlay)
  const drawConflicts = (ctx, time) => {
    if (conflictMap[time]) {
      ctx.fillStyle = "rgba(255, 0, 0, 0.5)";
      conflictMap[time].forEach(([y, x]) => {
        ctx.fillRect(x * CELL_SIZE, y * CELL_SIZE, CELL_SIZE, CELL_SIZE);
      });
    }
  };

  // Draw agents as colored circles
  const drawAgents = (ctx) => {
    agents.forEach((agent, i) => {
      if (!agent.path || agent.path.length === 0) return;
      // Current step position or last step if finished
      const step = Math.min(agent.step, agent.path.length - 1);
      const [y, x] = agent.path[step];
      ctx.fillStyle = AGENT_COLORS[i % AGENT_COLORS.length];
      ctx.beginPath();
      ctx.arc(
        x * CELL_SIZE + CELL_SIZE / 2,
        y * CELL_SIZE + CELL_SIZE / 2,
        CELL_SIZE / 4,
        0,
        Math.PI * 2
      );
      ctx.fill();
    });
  };

  // Animate agents step by step
  useEffect(() => {
    if (!isRunning) return;

    const ctx = canvasRef.current.getContext("2d");
    drawGrid(ctx);
    drawConflicts(ctx, currentTime);
    drawAgents(ctx);

    // Update agents' steps if simulation running
    let stillMoving = false;
    const updatedAgents = agents.map((agent) => {
      if (agent.step < agent.path.length - 1) {
        stillMoving = true;
        return { ...agent, step: agent.step + 1 };
      }
      return agent;
    });
    setAgents(updatedAgents);

    if (stillMoving) {
      const timer = setTimeout(() => {
        setCurrentTime((t) => t + 1);
      }, 500);
      return () => clearTimeout(timer);
    } else {
      setIsRunning(false); // stop simulation when all agents finish
    }
  }, [currentTime, isRunning, agents]);

  // Start simulation by calling backend and initializing state
  const startSimulation = async () => {
    setIsRunning(false);
    setCurrentTime(0);
    setAgents([]);
    setConflictMap({});

    // Example payload (replace or expand as needed)
    const payload = {
      agents: [
        { id: "A", start: [0, 0], goal: [9, 9] },
        { id: "B", start: [0, 9], goal: [9, 0] },
        { id: "C", start: [9, 0], goal: [0, 9] },
        { id: "D", start: [9, 9], goal: [0, 0] },
        { id: "E", start: [5, 0], goal: [5, 9] }
      ]
    };

    try {
      const response = await fetch("http://localhost:5000/compute-paths", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify(payload),
      });
      const data = await response.json();

      // Initialize agents with step = 0
      const loadedAgents = data.agents.map((agent) => ({
        ...agent,
        step: 0,
      }));

      setAgents(loadedAgents);
      setConflictMap(data.conflicts || {});
      setIsRunning(true);
    } catch (err) {
      console.error("Error fetching paths:", err);
      alert("Failed to fetch paths from backend.");
    }
  };

  return (
    <div style={{ textAlign: "center", marginTop: 20 }}>
    <h2>Warehouse Simulation (Hybrid Floyd)</h2>
    <canvas
      ref={canvasRef}
      width={CANVAS_WIDTH}
      height={CANVAS_HEIGHT}
      style={{ border: "2px solid #444", backgroundColor: "#f0f0f0" }}
    />
    <p style={{ fontSize: "18px", marginTop: "10px" }}>
      ⏱️ Time: <strong>{currentTime}</strong>
    </p>
    <button onClick={startSimulation} disabled={isRunning}>
      {isRunning ? "Running..." : "Start Simulation"}
    </button>
    <p>
      <span style={{ color: "red" }}>🟥 Red cells</span>: Potential time-window
      conflicts (overlapping agent occupation)
    </p>
  </div>
  );
}

export default App;
