import uvicorn
import os
import sys

if __name__ == "__main__":
    print("=============================================================")
    print("  GEOSCAN FleetCommander AI - Starting Server...")
    print("  Web Interface: http://localhost:8000")
    print("  API Docs:      http://localhost:8000/docs")
    print("=============================================================")
    uvicorn.run("backend.main:app", host="0.0.0.0", port=8000, reload=True)
