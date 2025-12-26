from pydantic_settings import BaseSettings, SettingsConfigDict
from typing import Optional

class Settings(BaseSettings):
    """
    Application settings loaded from .env file.
    """
    model_config = SettingsConfigDict(env_file="../.env", env_file_encoding='utf-8', extra='ignore')

    # API Keys
    OPENAI_API_KEY: Optional[str] = None
    GEMINI_API_KEY: Optional[str] = None

    # Qdrant Configuration
    QDRANT_URL: str
    QDRANT_API_KEY: str

    # Neon Postgres Configuration
    DATABASE_URL: Optional[str] = None

# Instantiate the settings
settings = Settings()

