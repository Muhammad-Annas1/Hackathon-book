import asyncpg
import logging
from src.core.settings import settings

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class PostgresService:
    """
    Service for logging chat history to Neon Serverless Postgres.
    """
    def __init__(self):
        self.database_url = settings.DATABASE_URL
        self._pool = None
        
    async def _get_connection(self):
        """Get a connection from the pool or create one."""
        if not self.database_url:
            logger.warning("DATABASE_URL not set. Skipping Postgres operations.")
            return None
        try:
            conn = await asyncpg.connect(self.database_url)
            return conn
        except Exception as e:
            logger.error(f"Failed to connect to Postgres: {e}")
            return None

    async def _ensure_table_exists(self, conn):
        """Create the chat_history table if it doesn't exist."""
        await conn.execute('''
            CREATE TABLE IF NOT EXISTS chat_history (
                id SERIAL PRIMARY KEY,
                user_message TEXT NOT NULL,
                bot_response TEXT NOT NULL,
                context TEXT,
                created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
            )
        ''')

    async def log_chat(self, user_message: str, bot_response: str, context: str = None):
        """
        Log a chat exchange to the database.
        This is designed to be called as a background task.
        """
        conn = await self._get_connection()
        if not conn:
            return
            
        try:
            await self._ensure_table_exists(conn)
            await conn.execute('''
                INSERT INTO chat_history (user_message, bot_response, context) 
                VALUES ($1, $2, $3)
            ''', user_message, bot_response, context)
            logger.info("Chat logged to Postgres successfully.")
        except Exception as e:
            logger.error(f"Failed to log chat: {e}")
        finally:
            await conn.close()
