from fastapi import FastAPI, HTTPException
from pydantic import BaseModel
from typing import List, Optional
import cohere
from qdrant_client import QdrantClient
from dotenv import load_dotenv
import os
import openai
from agents import Agent, Runner
from agents import function_tool
from fastapi.middleware.cors import CORSMiddleware

# Load environment variables
load_dotenv()

# Initialize clients
cohere_client = cohere.Client(os.getenv("COHERE"))
openai_client = openai.OpenAI(api_key=os.getenv("OPENAI_API_KEY"))

# Connect to Qdrant
qdrant = QdrantClient(
    url="https://7884d4a8-abf0-49d8-9416-5b1dc6d0d247.us-east4-0.gcp.cloud.qdrant.io",
    api_key=os.getenv("QDRANT"),
)

# Initialize FastAPI app
app = FastAPI(
    title="Robotech Book RAG API",
    description="RAG system for Physical AI & Humanoid Robotics Textbook",
    version="1.0.0"
)

# Add CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # In production, change this to your frontend URL
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Request/Response models
class QueryRequest(BaseModel):
    query: str
    selected_text: Optional[str] = None

class QueryResponse(BaseModel):
    answer: str
    sources: List[str]

class DocumentChunk(BaseModel):
    text: str
    url: str
    score: float

def get_embedding(text: str) -> List[float]:
    """Get embedding vector from Cohere Embed v3"""
    response = cohere_client.embed(
        model="embed-english-v3.0",
        input_type="search_query",  # Use search_query for queries
        texts=[text],
    )
    return response.embeddings[0]  # type: ignore

@function_tool
def retrieve(query: str) -> list[str]:
    """Retrieve documents from Qdrant using a query"""
    embedding = get_embedding(query)
    result = qdrant.query_points(
        collection_name="Hackathon-Book",
        query=embedding,
        limit=5
    )
    return [point.payload["text"] for point in result.points] #type: ignore

@app.get("/")
def read_root():
    return {"message": "Robotech Book RAG API"}

@app.post("/query", response_model=QueryResponse)
def query_endpoint(request: QueryRequest):
    try:
        # For now, use the direct OpenAI approach instead of agents to avoid potential issues
        # Get relevant documents from Qdrant
        embedding = get_embedding(request.query)
        try:
            result = qdrant.query_points(
                collection_name="Hackathon-Book",
                query=embedding,
                limit=5
            )

            # Format retrieved documents
            context_texts = [point.payload["text"] for point in result.points if point.payload and "text" in point.payload]  # type: ignore
            context = "\n\n".join(context_texts)

            # Extract URLs as sources
            sources = [point.payload["url"] for point in result.points if point.payload and "url" in point.payload]  # type: ignore
        except Exception as retrieval_error:
            print(f"Error retrieving from Qdrant: {str(retrieval_error)}")
            # Fallback: Use a general response without context
            context = "You are an AI tutor for the Physical AI & Humanoid Robotics textbook."
            sources = []

        # Create a message for OpenAI
        system_message = f"""
        You are an AI tutor for the Physical AI & Humanoid Robotics textbook.
        Use the following context to answer the user's question:
        {context}

        If the answer is not in the provided context, say "I don't know based on the textbook content."
        Be helpful and provide accurate information based on the textbook.
        """

        response = openai_client.chat.completions.create(
            model="gpt-3.5-turbo",
            messages=[
                {"role": "system", "content": system_message},
                {"role": "user", "content": request.query}
            ],
            max_tokens=500,
            temperature=0.3
        )

        answer = response.choices[0].message.content or "I couldn't generate a response based on the textbook content."

        return QueryResponse(answer=answer, sources=sources)
    except Exception as e:
        print(f"Error in /query endpoint: {str(e)}")  # Log the error
        raise HTTPException(status_code=500, detail=f"Error processing query: {str(e)}")

@app.post("/query-openai", response_model=QueryResponse)
def query_openai_endpoint(request: QueryRequest):
    """Alternative endpoint using OpenAI directly"""
    try:
        # Get relevant documents from Qdrant
        embedding = get_embedding(request.query)
        try:
            result = qdrant.query_points(
                collection_name="Hackathon-Book",
                query=embedding,
                limit=5
            )

            # Format retrieved documents with safety check
            context_texts = [point.payload["text"] for point in result.points if point.payload and "text" in point.payload]  # type: ignore
            context = "\n\n".join(context_texts)

            # Extract URLs as sources with safety check
            sources = [point.payload["url"] for point in result.points if point.payload and "url" in point.payload]  # type: ignore
        except Exception as retrieval_error:
            print(f"Error retrieving from Qdrant: {str(retrieval_error)}")
            # Fallback: Use a general response without context
            context = "You are an AI tutor for the Physical AI & Humanoid Robotics textbook."
            sources = []

        # Create a message for OpenAI
        system_message = f"""
        You are an AI tutor for the Physical AI & Humanoid Robotics textbook.
        Use the following context to answer the user's question:
        {context}

        If the answer is not in the provided context, say "I don't know based on the textbook content."
        """

        response = openai_client.chat.completions.create(
            model="gpt-3.5-turbo",
            messages=[
                {"role": "system", "content": system_message},
                {"role": "user", "content": request.query}
            ],
            max_tokens=500,
            temperature=0.3
        )

        answer = response.choices[0].message.content or "I couldn't generate a response based on the textbook content."

        return QueryResponse(answer=answer, sources=sources)
    except Exception as e:
        print(f"Error in /query-openai endpoint: {str(e)}")  # Log the error
        raise HTTPException(status_code=500, detail=f"Error processing query with OpenAI: {str(e)}")

@app.get("/health")
def health_check():
    return {"status": "healthy"}