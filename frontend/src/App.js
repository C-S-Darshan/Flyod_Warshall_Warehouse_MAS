import React, { useState, useEffect, useRef } from 'react';
import './App.css';

// --- Default Configuration ---
const DEFAULT_CONFIG = {
  rows: 10,
  cols: 10,
  shelves: [
    [2, 2], [2, 3], 
    [4, 1], [4, 2],
    [4, 6], [4, 7], [4, 8],
    [7, 2], [7, 3],
    [7, 6], [7, 7]
  ],
  agents: [
    { id: "A", start: [0, 0], goal: [9, 9] },
    { id: "B", start: [0, 9], goal: [9, 0] },
    { id: "C", start: [5, 0], goal: [5, 9] },
    { id: "D", start: [9, 4], goal: [0, 4] },
    { id: "E", start: [8, 0], goal: [1, 8] },
  ],
};

const COLORS = ["#e74c3c", "#3498db", "#2ecc71", "#f1c40f", "#9b59b6", "#e84393"];
const CELL_SIZE = 55; // Increased for better visuals

// --- SVG Icons ---
const PlayIcon = () => (
  <svg width="24" height="24" viewBox="0 0 24 24" fill="currentColor"><path d="M8 5v14l11-7z"></path></svg>
);
const PauseIcon = () => (
  <svg width="24" height="24" viewBox="0 0 24 24" fill="currentColor"><path d="M6 19h4V5H6v14zm8-14v14h4V5h-4z"></path></svg>
);
const ReplayIcon = () => (
  <svg width="24" height="24" viewBox="0 0 24 24" fill="currentColor"><path d="M12 5V1L7 6l5 5V7c3.31 0 6 2.69 6 6s-2.69 6-6 6-6-2.69-6-6H4c0 4.42 3.58 8 8 8s8-3.58 8-8-3.58-8-8-8z"></path></svg>
);
const RunIcon = () => (
    <svg width="24" height="24" viewBox="0 0 24 24" fill="currentColor"><path d="M12 2C6.48 2 2 6.48 2 12s4.48 10 10 10 10-4.48 10-10S17.52 2 12 2zm-2 14.5v-9l6 4.5-6 4.5z"></path></svg>
);

// --- Main App Component ---
function App() {
  const [simulationData, setSimulationData] = useState(null);
  const [time, setTime] = useState(0);
  const [isPlaying, setIsPlaying] = useState(false);
  const [isLoading, setIsLoading] = useState(false);
  const [logs, setLogs] = useState(["Ready to run simulation."]);
  const animationFrameId = useRef(null);

  // --- API Call ---
  const runSimulation = async () => {
    setIsLoading(true);
    setSimulationData(null);
    setIsPlaying(false);
    setTime(0);
    setLogs(["Running simulation..."]);
    try {
      const response = await fetch('http://127.0.0.1:8000/run_simulation', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(DEFAULT_CONFIG),
      });
      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }
      const data = await response.json();
      setSimulationData(data);
      setLogs(data.logs);
    } catch (error) {
      console.error("Failed to run simulation:", error);
      setLogs([`Error: Could not connect to the backend. Is it running?`, error.message]);
    } finally {
      setIsLoading(false);
    }
  };

  // --- Animation Logic ---
  useEffect(() => {
    if (isPlaying && simulationData) {
      animationFrameId.current = setInterval(() => {
        setTime(prevTime => {
          if (prevTime >= simulationData.max_time - 1) {
            setIsPlaying(false);
            return prevTime;
          }
          return prevTime + 1;
        });
      }, 500); // Slightly faster animation
    }
    return () => {
      clearInterval(animationFrameId.current);
    };
  }, [isPlaying, simulationData]);

  // --- Event Handlers ---
  const handlePlayPause = () => {
    if (simulationData) {
      setIsPlaying(!isPlaying);
    }
  };

  const handleReset = () => {
    setIsPlaying(false);
    setTime(0);
  };
  
  const handleSliderChange = (e) => {
    setIsPlaying(false);
    setTime(Number(e.target.value));
  };

  // --- Helper Functions ---
  const getLogClass = (log) => {
    if (log.startsWith('✅')) return 'log-success';
    if (log.startsWith('⚠️')) return 'log-warning';
    if (log.startsWith('❌')) return 'log-error';
    if (log.startsWith('🔄')) return 'log-info';
    return '';
  };

  // --- Render Functions ---
  const renderGrid = () => {
    const { rows, cols, shelves } = DEFAULT_CONFIG;
    const width = cols * CELL_SIZE;
    const height = rows * CELL_SIZE;

    return (
      <div className="grid-container">
        <svg width={width} height={height} className="grid-svg">
          {/* Grid Lines */}
          {Array.from({ length: rows + 1 }).map((_, i) => (
            <line key={`h-${i}`} x1={0} y1={i * CELL_SIZE} x2={width} y2={i * CELL_SIZE} className="grid-line" />
          ))}
          {Array.from({ length: cols + 1 }).map((_, i) => (
            <line key={`v-${i}`} x1={i * CELL_SIZE} y1={0} x2={i * CELL_SIZE} y2={height} className="grid-line" />
          ))}
          {/* Shelves */}
          {shelves.map(([r, c], i) => (
            <rect key={i} x={c * CELL_SIZE} y={r * CELL_SIZE} width={CELL_SIZE} height={CELL_SIZE} className="shelf" />
          ))}
          {/* Render paths and agents if data exists */}
          {simulationData && renderSimulationLayer()}
        </svg>
      </div>
    );
  };

  const renderSimulationLayer = () => {
    const { agents_data, conflicts } = simulationData;
    
    const pathToSvg = (path) => path.map(([r,c]) => `${c * CELL_SIZE + CELL_SIZE / 2},${r * CELL_SIZE + CELL_SIZE / 2}`).join(' ');

    return (
      <g>
        {/* Initial Floyd Paths (Dotted) */}
        {agents_data.map((agent, i) => (
          <polyline 
            key={`init-${agent.id}`} 
            points={pathToSvg(agent.initial_path)} 
            fill="none"
            className="initial-path" 
            style={{ stroke: COLORS[i % COLORS.length] }} 
          />
        ))}

        {/* Traveled Paths */}
        {agents_data.map((agent, i) => {
          if (!agent.path.length) return null;
          let solidPath = agent.path.slice(0, time + 1);
          let dashedPath = [];

          if (agent.rerouted) {
            const rerouteTime = agent.reroute_time;
            if (time >= rerouteTime) {
                solidPath = agent.original_segment.slice(0, rerouteTime + 1);
                dashedPath = agent.dijkstra_segment.slice(0, time - rerouteTime + 1);
            } else {
                solidPath = agent.original_segment.slice(0, time + 1);
            }
          }
          
          return (
            <g key={`travel-${agent.id}`}>
              <polyline 
                points={pathToSvg(solidPath)} 
                fill="none"
                className="traveled-path-solid" 
                style={{ stroke: COLORS[i % COLORS.length] }} 
              />
              {dashedPath.length > 0 && 
                <polyline 
                  points={pathToSvg(dashedPath)} 
                  fill="none"
                  className="traveled-path-dashed" 
                  style={{ stroke: COLORS[i % COLORS.length] }} 
                />
              }
            </g>
          );
        })}
        
        {/* Conflict Markers */}
        {conflicts[time] && conflicts[time].map(([r, c], i) => (
            <rect key={`c-${i}`} x={c * CELL_SIZE} y={r * CELL_SIZE} width={CELL_SIZE} height={CELL_SIZE} className="conflict-marker" />
        ))}

        {/* Agents */}
        {agents_data.map((agent, i) => {
          if (!agent.path.length) return null;
          const posIndex = Math.min(time, agent.path.length - 1);
          const [r, c] = agent.path[posIndex];
          return (
            <g key={`agent-${agent.id}`} transform={`translate(${c * CELL_SIZE + CELL_SIZE / 2}, ${r * CELL_SIZE + CELL_SIZE / 2})`}>
              <circle r={CELL_SIZE / 2.5} style={{ fill: COLORS[i % COLORS.length] }} className="agent-dot" />
              <text y="6" textAnchor="middle" className="agent-label">{agent.id}</text>
            </g>
          );
        })}
      </g>
    );
  };

  return (
    <div className="App">
      <header className="App-header">
        <h1>Hybrid Floyd-Dijkstra Warehouse Simulation</h1>
      </header>
      <main className="main-container">
        <div className="simulation-panel panel">
          {renderGrid()}
          <div className="controls">
            <button onClick={runSimulation} disabled={isLoading} className="control-button">
              <RunIcon />
              {isLoading ? 'Simulating...' : 'Run Simulation'}
            </button>
            <button onClick={handlePlayPause} disabled={!simulationData} className="control-button">
              {isPlaying ? <PauseIcon /> : <PlayIcon />}
              {isPlaying ? 'Pause' : 'Play'}
            </button>
            <button onClick={handleReset} disabled={!simulationData} className="control-button">
              <ReplayIcon />
              Reset
            </button>
            <div className="slider-container">
              <input
                type="range"
                min="0"
                max={simulationData ? simulationData.max_time - 1 : 0}
                value={time}
                onChange={handleSliderChange}
                disabled={!simulationData}
                className="slider"
              />
              <span>{time} / {simulationData ? simulationData.max_time - 1 : 0}</span>
            </div>
          </div>
        </div>
        <div className="logs-panel panel">
          <h2>Simulation Logs</h2>
          <div className="logs-content">
            {logs.map((log, i) => <p key={i} className={getLogClass(log)}>{log}</p>)}
          </div>
        </div>
      </main>
    </div>
  );
}

export default App;
