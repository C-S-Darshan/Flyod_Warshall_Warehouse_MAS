from fastapi import FastAPI, Request
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from typing import List, Dict
from hybrid_floyd import compute_paths

app = FastAPI()

# Enable CORS for local React frontend (default: http://localhost:3000)
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # For development only; restrict in prod
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

class Agent(BaseModel):
    id: str
    start: List[int]
    goal: List[int]

class AgentRequest(BaseModel):
    agents: List[Agent]

@app.post("/compute-paths")
async def compute(agent_req: AgentRequest):
    data = {"agents": [agent.dict() for agent in agent_req.agents]}
    result = compute_paths(data)
    return result
