import React, { useState, useEffect, useRef } from 'react';
import './App.css'; // Import the CSS file

const API_BASE_URL = 'http://127.0.0.1:8000'; // Replace with your FastAPI backend URL

function App() {
  const [simulationData, setSimulationData] = useState(null);
  const [currentTime, setCurrentTime] = useState(0);
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState(null);
  const [logs, setLogs] = useState([]);

  // Default simulation configuration (matches backend defaults)
  const defaultSimulationConfig = {
    rows: 10,
    cols: 10,
    shelves: [
      [4, 4], [4, 5], [5, 4], [5, 5],
      [2, 2], [2, 3], [2, 4],
      [7, 6], [7, 7], [7, 8]
    ],
    agents: [
      { id: "A", start: [0, 0], goal: [9, 9] },
      { id: "B", start: [0, 9], goal: [9, 0] },
      { id: "C", start: [9, 0], goal: [0, 9] },
      { id: "D", start: [9, 9], goal: [0, 0] },
      { id: "E", start: [5, 0], goal: [5, 9] },
    ],
  };

  // Define colors directly as hex codes for CSS use
  const agentColors = ["#ef4444", "#3b82f6", "#22c55e", "#f97316", "#a855f7"]; // red, blue, green, orange, purple

  // Function to fetch simulation data from backend
  const fetchSimulation = async () => {
    setIsLoading(true);
    setError(null);
    try {
      const response = await fetch(`${API_BASE_URL}/simulate`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify(defaultSimulationConfig),
      });

      if (!response.ok) {
        const errorData = await response.json();
        throw new Error(errorData.detail || 'Failed to fetch simulation data');
      }

      const data = await response.json();
      setSimulationData(data);
      setCurrentTime(0); // Reset time when new simulation loads
      setLogs(data.conflict_logs);
    } catch (err) {
      setError(err.message);
      console.error("Error fetching simulation:", err);
    } finally {
      setIsLoading(false);
    }
  };

  // Fetch simulation data on component mount
  useEffect(() => {
    fetchSimulation();
  }, []);

  if (isLoading) {
    return (
      <div className="loading-container">
        <p className="loading-text">Loading simulation...</p>
      </div>
    );
  }

  if (error) {
    return (
      <div className="error-container">
        <p className="error-text">Error: {error}</p>
        <p className="error-subtext">Please ensure the FastAPI backend is running at {API_BASE_URL}</p>
        <button
          onClick={fetchSimulation}
          className="error-button"
        >
          Try Again
        </button>
      </div>
    );
  }

  if (!simulationData) {
    return null; // Or a placeholder if data is not yet available
  }

  const { rows, cols } = simulationData.grid_size;
  const cellSize = 50; // Size of each cell in pixels
  const svgWidth = cols * cellSize;
  const svgHeight = rows * cellSize;

  // Function to convert grid coordinates to SVG coordinates
  const toSvgCoords = (r, c) => ({
    x: c * cellSize + cellSize / 2,
    y: r * cellSize + cellSize / 2,
  });

  // Calculate coordinates for drawing lines
  const getLineCoords = (pathCoords) => {
    if (!pathCoords || pathCoords.length < 2) return null;
    let d = "";
    pathCoords.forEach((coord, index) => {
      const { x, y } = toSvgCoords(coord[0], coord[1]);
      if (index === 0) {
        d += `M ${x} ${y}`;
      } else {
        d += ` L ${x} ${y}`;
      }
    });
    return d;
  };

  // Get current positions of agents
  const currentAgentPositions = simulationData.agents_data.map(agent => {
    const path = agent.path;
    if (currentTime < path.length) {
      return path[currentTime];
    }
    return path[path.length - 1]; // Stay at destination
  });

  // Get current conflict nodes
  const currentConflicts = simulationData.conflict_map[currentTime] || [];

  return (
    <div className="app-container">
      <h1 className="app-title">Warehouse Simulation</h1>

      <div className="main-content">
        {/* Simulation Grid */}
        <div className="simulation-grid-container">
          <svg width={svgWidth} height={svgHeight} className="simulation-svg">
            {/* Grid Lines */}
            {Array.from({ length: rows + 1 }).map((_, i) => (
              <line key={`h-${i}`} x1="0" y1={i * cellSize} x2={svgWidth} y2={i * cellSize} stroke="#e0e0e0" strokeWidth="1" />
            ))}
            {Array.from({ length: cols + 1 }).map((_, i) => (
              <line key={`v-${i}`} x1={i * cellSize} y1="0" x2={i * cellSize} y2={svgHeight} stroke="#e0e0e0" strokeWidth="1" />
            ))}

            {/* Shelves */}
            {simulationData.shelves.map((shelf, index) => (
              <rect
                key={`shelf-${index}`}
                x={shelf[1] * cellSize}
                y={shelf[0] * cellSize}
                width={cellSize}
                height={cellSize}
                fill="#333"
                rx="5" ry="5" // Rounded corners
              />
            ))}

            {/* Floyd Paths (background, dotted) */}
            {defaultSimulationConfig.agents.map((agentConfig, i) => {
                const color = agentColors[i % agentColors.length];
                // Note: Re-calculating Floyd path here for visualization.
                // In a more complex app, this might come from the backend too.
                // Ensure shelves are stringified for Set lookup
                const tempAdj = build_graph_for_frontend(rows, cols, new Set(defaultSimulationConfig.shelves.map(s => JSON.stringify(s))));
                const [, pathMatrix] = floyd_with_path_for_frontend(tempAdj);
                const initialFloydPathIds = reconstruct_path_for_frontend(
                    agentConfig.start[0] * cols + agentConfig.start[1],
                    agentConfig.goal[0] * cols + agentConfig.goal[1],
                    pathMatrix
                );
                const floydPathCoords = initialFloydPathIds.map(id => [Math.floor(id / cols), id % cols]);
                const d = getLineCoords(floydPathCoords);
                return d ? (
                    <path
                        key={`floyd-path-${agentConfig.id}`}
                        d={d}
                        fill="none"
                        stroke={color}
                        strokeWidth="1"
                        strokeDasharray="2 2"
                        opacity="0.2"
                    />
                ) : null;
            })}

            {/* Agent Paths (dynamic) */}
            {simulationData.agents_data.map((agent, i) => {
              const color = agentColors[i % agentColors.length];
              const rerouted = agent.rerouted;
              const rerouteTime = agent.reroute_time;

              // Solid path segment (original path or part of it)
              let solidPathCoords = [];
              if (rerouted) {
                solidPathCoords = agent.original_segment;
              } else {
                solidPathCoords = agent.path.slice(0, currentTime + 1);
              }
              const solidD = getLineCoords(solidPathCoords);

              // Dashed path segment (Dijkstra reroute)
              let dashedPathCoords = [];
              if (rerouted && currentTime >= rerouteTime) {
                // The Dijkstra segment starts at rerouteTime
                const idxInDijkstra = currentTime - rerouteTime;
                if (idxInDijkstra >= 0 && idxInDijkstra < agent.dijkstra_segment.length) {
                    dashedPathCoords = agent.dijkstra_segment.slice(0, idxInDijkstra + 1);
                } else {
                    dashedPathCoords = agent.dijkstra_segment; // Show full segment if frame exceeds
                }
              }
              const dashedD = getLineCoords(dashedPathCoords);

              return (
                <React.Fragment key={`agent-paths-${agent.id}`}>
                  {solidD && (
                    <path
                      d={solidD}
                      fill="none"
                      stroke={color}
                      strokeWidth="2"
                      opacity="0.5"
                      strokeLinecap="round"
                      strokeLinejoin="round"
                    />
                  )}
                  {dashedD && (
                    <path
                      d={dashedD}
                      fill="none"
                      stroke={color}
                      strokeWidth="3"
                      strokeDasharray="8 4" // Thicker dashed line
                      opacity="0.8"
                      strokeLinecap="round"
                      strokeLinejoin="round"
                    />
                  )}
                </React.Fragment>
              );
            })}

            {/* Conflict Markers */}
            {currentConflicts.map((coord, index) => {
              const { x, y } = toSvgCoords(coord[0], coord[1]);
              return (
                <circle
                  key={`conflict-${index}`}
                  cx={x}
                  cy={y}
                  r={cellSize * 0.4} // Size of conflict marker
                  fill="none"
                  stroke="red"
                  strokeWidth="3"
                  className="conflict-marker" // Apply CSS class for animation
                />
              );
            })}

            {/* Agents */}
            {simulationData.agents_data.map((agent, i) => {
              const [r, c] = currentAgentPositions[i];
              const { x, y } = toSvgCoords(r, c);
              const color = agentColors[i % agentColors.length];
              return (
                <g key={`agent-${agent.id}`} transform={`translate(${x}, ${y})`}>
                  <circle
                    cx="0"
                    cy="0"
                    r={cellSize * 0.3} // Agent size
                    fill={color}
                    stroke="white"
                    strokeWidth="1"
                    className="agent-dot" // Apply CSS class for transition
                  />
                  <text
                    x="0"
                    y="0"
                    textAnchor="middle"
                    dominantBaseline="middle"
                    fontSize="10"
                    fill="white"
                    fontWeight="bold"
                  >
                    {agent.id}
                  </text>
                </g>
              );
            })}
          </svg>
        </div>

        {/* Controls and Logs */}
        <div className="controls-logs-container">
          <div className="section-box">
            <h2 className="section-title">Timeline</h2>
            <input
              type="range"
              min="0"
              max={simulationData.max_time_for_animation}
              value={currentTime}
              onChange={(e) => setCurrentTime(parseInt(e.target.value))}
              className="timeline-slider"
            />
            <div className="time-display">Time: {currentTime} / {simulationData.max_time_for_animation}</div>
          </div>

          <div className="section-box">
            <h2 className="section-title">Simulation Controls</h2>
            <button
              onClick={() => setCurrentTime(0)}
              className="control-button reset-button"
            >
              Reset Simulation
            </button>
            <button
              onClick={fetchSimulation}
              className="control-button rerun-button"
            >
              Rerun Simulation
            </button>
          </div>

          <div className="section-box">
            <h2 className="section-title">Simulation Logs</h2>
            <div className="logs-box">
              {logs.length > 0 ? (
                logs.map((log, index) => (
                  <p key={index} className="log-entry">{log}</p>
                ))
              ) : (
                <p>No logs yet.</p>
              )}
            </div>
          </div>
        </div>
      </div>
    </div>
  );
}

// --- Helper functions (replicated from backend for client-side Floyd path visualization) ---
// These are necessary because the frontend needs to calculate the initial Floyd paths
// to draw them as background elements, independent of the simulation data from the backend.

let _frontend_rows_global = 0; // Renamed to avoid conflict with function parameters
let _frontend_cols_global = 0; // Renamed to avoid conflict with function parameters

function build_graph_for_frontend(rows, cols, shelves) {
    _frontend_rows_global = rows;
    _frontend_cols_global = cols;
    const N = rows * cols;
    // Corrected: Initialize 2D array with JavaScript methods
    const adj = Array.from({ length: N }, () => Array(N).fill(Infinity));
    
    const DIRS = [
        [-1, 0], [1, 0], [0, -1], [0, 1]
    ];

    for (let x = 0; x < rows; x++) {
        for (let y = 0; y < cols; y++) {
            if (shelves.has(JSON.stringify([x, y]))) { // Convert tuple to string for Set lookup
                continue;
            }
            const a = x * cols + y;
            adj[a][a] = 0;

            for (const [dx, dy] of DIRS) {
                const nx = x + dx;
                const ny = y + dy;
                if (nx >= 0 && nx < rows && ny >= 0 && ny < cols && !shelves.has(JSON.stringify([nx, ny]))) {
                    const b = nx * cols + ny;
                    adj[a][b] = 1;
                }
            }
        }
    }
    return adj;
}

function floyd_with_path_for_frontend(adj) {
    const N = adj.length;
    const dist = adj.map(row => [...row]); // Deep copy using map and spread
    const path = Array.from({ length: N }, () => Array(N).fill(-1)); // Initialize with -1

    for (let i = 0; i < N; i++) {
        for (let j = 0; j < N; j++) {
            if (adj[i][j] === 1) {
                path[i][j] = j;
            }
        }
    }

    for (let k = 0; k < N; k++) {
        for (let i = 0; i < N; i++) {
            for (let j = 0; j < N; j++) {
                if (dist[i][k] !== Infinity && dist[k][j] !== Infinity) {
                    if (dist[i][k] + dist[k][j] < dist[i][j]) {
                        dist[i][j] = dist[i][k] + dist[k][j];
                        path[i][j] = path[i][k];
                    }
                }
            }
        }
    }
    return [dist, path];
}

function reconstruct_path_for_frontend(i, j, path_matrix) { // Removed 'cols' as it's not needed here
    if (i === j) {
        return [i];
    }
    if (path_matrix[i][j] === -1) {
        return [];
    }

    const path = [i];
    let current = i;
    // Use _frontend_cols_global for node_id and id_to_coord within this context
    const node_id_frontend = (x, y) => x * _frontend_cols_global + y;
    const id_to_coord_frontend = (id_) => [Math.floor(id_ / _frontend_cols_global), id_ % _frontend_cols_global];

    while (current !== j) {
        const next_node = path_matrix[current][j];
        if (next_node === -1) {
            return [];
        }
        path.push(next_node);
        current = next_node;
    }
    return path;
}

export default App;
