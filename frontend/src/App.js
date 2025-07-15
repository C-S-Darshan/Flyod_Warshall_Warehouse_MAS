import React, { useRef, useState, useEffect } from "react";

const CELL_SIZE = 40;
const ROWS = 10;
const COLS = 10;
const CANVAS_WIDTH = CELL_SIZE * COLS;
const CANVAS_HEIGHT = CELL_SIZE * ROWS;
const AGENT_COLORS = ["red", "blue", "green", "orange", "purple"];

// Convert CSS color name to RGB components
function getRGBFromColor(name) {
  const canvas = document.createElement("canvas");
  const ctx = canvas.getContext("2d");
  ctx.fillStyle = name;
  document.body.appendChild(canvas); // needed for computed style
  const computed = getComputedStyle(canvas).color;
  document.body.removeChild(canvas);
  const match = computed.match(/\d+/g);
  return match ? match.map(Number) : [0, 0, 0];
}

// 📦 Shelves (obstacles)
const SHELVES = new Set([
  "4,4", "4,5", "5,4", "5,5",
  "2,2", "2,3", "2,4",
  "7,6", "7,7", "7,8"
]);

function App() {
  const canvasRef = useRef(null);
  const [agents, setAgents] = useState([]);
  const [conflictMap, setConflictMap] = useState({});
  const [currentTime, setCurrentTime] = useState(0);
  const [isSimLoaded, setIsSimLoaded] = useState(false);

  // 🎨 Draw everything on canvas
  // 🎨 Draw everything on canvas
  useEffect(() => {
  const ctx = canvasRef.current.getContext("2d");
  ctx.clearRect(0, 0, CANVAS_WIDTH, CANVAS_HEIGHT);

  // Draw grid
  ctx.strokeStyle = "#aaa";
  for (let r = 0; r < ROWS; r++) {
    for (let c = 0; c < COLS; c++) {
      ctx.strokeRect(c * CELL_SIZE, r * CELL_SIZE, CELL_SIZE, CELL_SIZE);
    }
  }

  // Draw shelves
  SHELVES.forEach((coord) => {
    const [y, x] = coord.split(",").map(Number);
    ctx.fillStyle = "#666";
    ctx.fillRect(x * CELL_SIZE, y * CELL_SIZE, CELL_SIZE, CELL_SIZE);
  });

  // Draw agent paths (trails with agent color)
  agents.forEach((agent, i) => {
    const [r, g, b] = getRGBFromColor(AGENT_COLORS[i % AGENT_COLORS.length]);

    // Trail fill
    ctx.fillStyle = `rgba(${r}, ${g}, ${b}, 0.2)`;
    agent.path.forEach(([y, x]) => {
      ctx.fillRect(x * CELL_SIZE, y * CELL_SIZE, CELL_SIZE, CELL_SIZE);
    });

    // Optional dashed stroke if rerouted
    if (agent.rerouted) {
      ctx.strokeStyle = `rgba(${r}, ${g}, ${b}, 0.8)`;
      ctx.setLineDash([4, 4]);
      agent.path.forEach(([y, x]) => {
        ctx.strokeRect(x * CELL_SIZE, y * CELL_SIZE, CELL_SIZE, CELL_SIZE);
      });
      ctx.setLineDash([]);
    }
  });

  // Draw time-window conflicts
  if (conflictMap[currentTime]) {
    ctx.fillStyle = "rgba(255, 0, 0, 0.5)";
    conflictMap[currentTime].forEach(([y, x]) => {
      ctx.fillRect(x * CELL_SIZE, y * CELL_SIZE, CELL_SIZE, CELL_SIZE);
    });
  }

  // Draw agents
  agents.forEach((agent, i) => {
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
}, [currentTime, agents, conflictMap]);



  // 🟢 Load simulation paths
  const startSimulation = async () => {
    setIsSimLoaded(false);
    setCurrentTime(0);
    setAgents([]);
    setConflictMap({});

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
      const res = await fetch("http://localhost:5000/compute-paths", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify(payload),
      });
      const data = await res.json();

      const initializedAgents = data.agents.map((agent) => ({
        ...agent,
        step: 0,
        rerouted: agent.rerouted || false,
      }));


      setAgents(initializedAgents);
      setConflictMap(data.conflicts || {});
      setIsSimLoaded(true);
    } catch (err) {
      console.error("Failed to fetch paths:", err);
      alert("💥 Backend not working or JSON broken");
    }
  };

  // 🔁 Go to next time step
  const nextStep = () => {
    if (!isSimLoaded) return;

    const updatedAgents = agents.map((agent) => {
      if (agent.step < agent.path.length - 1) {
        return { ...agent, step: agent.step + 1 };
      }
      return agent;
    });

    setAgents(updatedAgents);
    setCurrentTime((t) => t + 1);
  };

  return (
    <div style={{ textAlign: "center", marginTop: 20 }}>
      <h2>📦 Warehouse Simulation (Hybrid Floyd)</h2>
      <canvas
        ref={canvasRef}
        width={CANVAS_WIDTH}
        height={CANVAS_HEIGHT}
        style={{ border: "2px solid #444", backgroundColor: "#f0f0f0" }}
      />
      <p style={{ fontSize: "18px", marginTop: "10px" }}>
        ⏱️ Time: <strong>{currentTime}</strong>
      </p>

      <div style={{ marginTop: 10 }}>
        <button onClick={startSimulation}>Start Simulation</button>
        <button
          onClick={nextStep}
          style={{ marginLeft: 10 }}
          disabled={!isSimLoaded}
        >
          Next Step
        </button>
      </div>

      <p style={{ marginTop: 20 }}>
        <span style={{ color: "red" }}>🟥 Red</span>: Time-window conflicts<br />
        <span style={{ color: "gray" }}>⬛ Gray</span>: Shelves (obstacles)<br />
        <span style={{ color: "blue" }}>🟦 Blue</span>: Agent path trail
      </p>
    </div>
  );
}

export default App;
