#!/usr/bin/env python3
"""
ROSIE Conversational AI System
Implements CONVERSE_B4M_OLLAMA_HYBRID specification

A voice-controlled conversational AI that combines:
- Local Ollama for immediate responses (<1 second)
- bike4mind API for background intelligence (5-10 seconds)
- Whisper for speech-to-text
- Piper for text-to-speech

Architecture: State machine with three states (LISTENING, RESPONDING, SPEAKING)
Wake word: "Rosie"
"""

import os
import sys
import signal
import subprocess
import threading
import time
import json
import re
import argparse
from enum import Enum
from pathlib import Path
import requests

# Global debug mode flag - controlled by --debug CLI argument
DEBUG_MODE = False

def debug_print(*args, **kwargs):
    """Print only if DEBUG_MODE is enabled"""
    if DEBUG_MODE:
        print(*args, **kwargs)

def configure_logging(debug_mode):
    """Configure logging levels based on debug mode"""
    import logging
    if not debug_mode:
        # Suppress noisy library logs in normal mode
        logging.getLogger("httpx").setLevel(logging.WARNING)
        logging.getLogger("httpcore").setLevel(logging.WARNING)
        logging.getLogger("chromadb").setLevel(logging.WARNING)
        logging.getLogger("numexpr").setLevel(logging.WARNING)


def sync_google_calendar(silent=False):
    """
    Sync Google Calendar events to knowledge base on startup.

    Handles authentication, including re-authentication if refresh token expired.

    Args:
        silent: If True, suppress most output (for background sync)

    Returns:
        True if sync successful, False otherwise
    """
    import subprocess
    import json
    from pathlib import Path

    rosie_dir = Path(__file__).parent.parent
    sync_script = rosie_dir / 'src' / 'rosie_calendar_sync.py'
    setup_script = rosie_dir / 'src' / 'rosie_calendar_setup.py'

    if not sync_script.exists():
        if not silent:
            print("[CALENDAR] Calendar sync script not found, skipping")
        return False

    # Check if calendar config exists
    config_file = rosie_dir / 'data' / 'calendar_config.json'
    if not config_file.exists():
        if not silent:
            print("[CALENDAR] No calendar configuration found, skipping sync")
            print("[CALENDAR] Run 'python3 src/rosie_calendar_setup.py' to configure calendars")
        return False

    if not silent:
        print("[CALENDAR] Syncing Google Calendar events...")

    try:
        # Try to run the sync script
        result = subprocess.run(
            ['python3', str(sync_script)],
            capture_output=True,
            text=True,
            timeout=60,
            cwd=str(rosie_dir)
        )

        # Check if auth failed with invalid_grant (refresh token expired)
        if 'invalid_grant' in result.stderr or 'RefreshError' in result.stderr:
            print("[CALENDAR] ⚠ Google Calendar credentials expired - re-authentication required")
            print("[CALENDAR] Opening browser for Google authentication...")

            # Run the setup script to re-authenticate
            auth_result = subprocess.run(
                ['python3', str(setup_script)],
                capture_output=False,  # Show output to user for browser auth
                text=True,
                timeout=120,  # Allow more time for browser auth
                cwd=str(rosie_dir)
            )

            if auth_result.returncode != 0:
                print("[CALENDAR] ✗ Re-authentication failed")
                return False

            # Retry sync after re-authentication
            print("[CALENDAR] Retrying calendar sync...")
            result = subprocess.run(
                ['python3', str(sync_script)],
                capture_output=True,
                text=True,
                timeout=60,
                cwd=str(rosie_dir)
            )

        if result.returncode == 0:
            # Extract event count from output
            import re
            match = re.search(r'Total events fetched: (\d+)', result.stdout)
            event_count = match.group(1) if match else "?"
            if not silent:
                print(f"[CALENDAR] ✓ Synced {event_count} events to knowledge base")
            return True
        else:
            if not silent:
                print(f"[CALENDAR] ✗ Sync failed: {result.stderr[:200] if result.stderr else 'Unknown error'}")
            return False

    except subprocess.TimeoutExpired:
        if not silent:
            print("[CALENDAR] ✗ Sync timed out")
        return False
    except Exception as e:
        if not silent:
            print(f"[CALENDAR] ✗ Sync error: {e}")
        return False


import whisper
import numpy as np
import sounddevice as sd
from dotenv import load_dotenv

# RAG imports
try:
    from llama_index.core import VectorStoreIndex, SimpleDirectoryReader, Settings, StorageContext
    from llama_index.core.node_parser import SentenceSplitter
    from llama_index.embeddings.ollama import OllamaEmbedding
    from llama_index.llms.ollama import Ollama
    from llama_index.vector_stores.chroma import ChromaVectorStore
    import chromadb
    RAG_AVAILABLE = True
except ImportError:
    RAG_AVAILABLE = False
    print("WARNING: RAG dependencies not installed. Knowledge base features disabled.")
    print("Install with: pip install llama-index llama-index-llms-ollama llama-index-embeddings-ollama llama-index-vector-stores-chroma chromadb")

# Load optional configuration from .env file (only if variables not already set)
# Environment variables from .bashrc always take precedence
# Look in the rosie root directory (parent of src/)
env_file = Path(__file__).parent.parent / '.env.rosie'
if env_file.exists():
    load_dotenv(env_file, override=False)


class RosieRAG:
    """RAG (Retrieval-Augmented Generation) knowledge base for ROSIE"""

    def __init__(self, knowledge_base_dir, chroma_db_dir, private_mode=False):
        """
        Initialize RAG system with LlamaIndex and ChromaDB

        Args:
            knowledge_base_dir: Path to directory containing .md files
            chroma_db_dir: Path to directory for ChromaDB persistence
            private_mode: If True, include private/ folder documents. If False, exclude them.
        """
        if not RAG_AVAILABLE:
            raise ImportError("RAG dependencies not installed")

        self.knowledge_base_dir = Path(knowledge_base_dir)
        self.chroma_db_dir = Path(chroma_db_dir)
        self.private_mode = private_mode
        self.index = None
        self.retriever = None

        # Create directories if they don't exist
        self.knowledge_base_dir.mkdir(exist_ok=True)
        self.chroma_db_dir.mkdir(exist_ok=True)

        debug_print(f"[RAG] Initializing knowledge base...")

        # Setup Ollama embeddings (no LLM needed for retrieval-only RAG)
        Settings.embed_model = OllamaEmbedding(
            model_name="nomic-embed-text",
            base_url="http://localhost:11434"
        )

        # Initialize ChromaDB
        chroma_client = chromadb.PersistentClient(path=str(self.chroma_db_dir))
        chroma_collection = chroma_client.get_or_create_collection("rosie_knowledge")
        vector_store = ChromaVectorStore(chroma_collection=chroma_collection)
        storage_context = StorageContext.from_defaults(vector_store=vector_store)

        # Check for .md files in knowledge base (recursively to include private/)
        md_files = list(self.knowledge_base_dir.rglob("*.md"))

        if not md_files:
            debug_print(f"[RAG] No .md files found in {self.knowledge_base_dir}")
            debug_print(f"[RAG] Knowledge base is empty. Add .md files and restart ROSIE.")
            # Create empty index
            self.index = VectorStoreIndex.from_documents(
                [],
                storage_context=storage_context
            )
        else:
            # Load and index documents
            try:
                # Build exclude pattern for private/ folder when not in private mode
                reader_kwargs = {
                    "input_dir": str(self.knowledge_base_dir),
                    "required_exts": [".md"],
                    "recursive": True
                }

                # Exclude private/ folder when private_mode is False
                if not self.private_mode:
                    reader_kwargs["exclude"] = ["**/private/*", "**/private/**/*"]

                documents = SimpleDirectoryReader(**reader_kwargs).load_data()
                debug_print(f"[RAG] Loaded {len(documents)} files")

                # Use larger chunk size to keep calendar days together
                # 512 chars fits 2-3 calendar entries per chunk
                text_splitter = SentenceSplitter(
                    chunk_size=512,  # Larger chunks to keep days together
                    chunk_overlap=100  # More overlap for context
                )

                # Create or update index with chunking
                self.index = VectorStoreIndex.from_documents(
                    documents,
                    storage_context=storage_context,
                    transformations=[text_splitter],
                    show_progress=False
                )
                debug_print(f"[RAG] Created index with chunked documents")

            except Exception as e:
                debug_print(f"[RAG] Error loading documents: {e}")
                # Create empty index on error
                self.index = VectorStoreIndex.from_documents(
                    [],
                    storage_context=storage_context
                )

        # Create retriever (NO LLM calls - embeddings only for fast retrieval)
        # This completely bypasses LLM synthesis and only uses vector similarity
        self.retriever = self.index.as_retriever(
            similarity_top_k=5
        )

        # Check if Ollama is using GPU by examining recent logs
        # Skip journalctl check (can cause hangs) - just print ready message
        debug_print(f"[RAG] Ready ({len(md_files)} knowledge base files)")

    def query(self, question, top_k=5, date_filter=None):
        """
        Retrieve relevant context for a question using ONLY embeddings (no LLM calls)

        Args:
            question: The question to search for
            top_k: Number of relevant chunks to retrieve (default: 5)
            date_filter: Optional date string to filter results (e.g., "December 13, 2025")
                         When provided, prioritizes chunks containing this date

        Returns:
            String containing relevant context, or empty string if no results
        """
        if not self.retriever:
            return ""

        try:
            # Update top_k if different from default
            if top_k != 5:
                self.retriever = self.index.as_retriever(
                    similarity_top_k=top_k
                )

            # Retrieve relevant nodes (NO LLM CALL - embeddings only)
            nodes = self.retriever.retrieve(question)

            # Extract text from retrieved nodes
            if nodes:
                debug_print(f"[RAG] Query: '{question}'")
                debug_print(f"[RAG] Retrieved {len(nodes)} chunks:")

                all_contexts = []
                date_filtered_contexts = []

                for i, node in enumerate(nodes):
                    # Get the text content from each node
                    text = node.node.get_content()
                    # Include source file info if available
                    source = node.node.metadata.get('file_name', 'unknown')
                    # Get similarity score if available
                    score = getattr(node, 'score', None)
                    score_str = f" (score: {score:.3f})" if score is not None else ""
                    debug_print(f"[RAG]   {i+1}. {source}{score_str}: {text[:80].replace(chr(10), ' ')}...")

                    context_entry = f"[From {source}]\n{text}"
                    all_contexts.append(context_entry)

                    # Check if this chunk contains the target date
                    if date_filter and date_filter.lower() in text.lower():
                        date_filtered_contexts.append(context_entry)
                        debug_print(f"[RAG]      ^ Contains target date: {date_filter}")

                # If we have date-filtered results, prioritize them
                if date_filtered_contexts:
                    debug_print(f"[RAG] Using {len(date_filtered_contexts)} date-filtered chunks (target: {date_filter})")
                    return "\n\n".join(date_filtered_contexts)
                else:
                    if date_filter:
                        debug_print(f"[RAG] No chunks matched date filter: {date_filter}, using all {len(all_contexts)} chunks")
                    return "\n\n".join(all_contexts)
            else:
                debug_print(f"[RAG] Query: '{question}' - No chunks retrieved")
                return ""

        except Exception as e:
            debug_print(f"[RAG] Query error: {e}")
            import traceback
            traceback.print_exc()
            return ""

    def reload_with_private_mode(self, private_mode):
        """
        Reload RAG system with new private mode setting

        Args:
            private_mode: New private mode setting (True/False)

        Returns:
            True if reload successful, False otherwise
        """
        try:
            debug_print(f"[RAG] Reloading with private_mode={private_mode}")

            # Update private mode flag
            self.private_mode = private_mode

            # Re-initialize ChromaDB (clearing old collection)
            chroma_client = chromadb.PersistentClient(path=str(self.chroma_db_dir))

            # Delete existing collection to force re-indexing
            try:
                chroma_client.delete_collection("rosie_knowledge")
                debug_print(f"[RAG] Deleted existing collection")
            except Exception as e:
                debug_print(f"[RAG] Collection didn't exist or couldn't delete: {e}")

            # Create new collection
            chroma_collection = chroma_client.get_or_create_collection("rosie_knowledge")
            vector_store = ChromaVectorStore(chroma_collection=chroma_collection)
            storage_context = StorageContext.from_defaults(vector_store=vector_store)

            # Load documents with new private mode setting
            debug_print(f"[RAG] Loading documents...")
            reader_kwargs = {
                "input_dir": str(self.knowledge_base_dir),
                "required_exts": [".md"],
                "recursive": True
            }

            # Exclude private/ folder when private_mode is False
            if not self.private_mode:
                reader_kwargs["exclude"] = ["**/private/*", "**/private/**/*"]
                debug_print(f"[RAG] Excluding private/ folder")
            else:
                debug_print(f"[RAG] Including private/ folder")

            documents = SimpleDirectoryReader(**reader_kwargs).load_data()

            debug_print(f"[RAG] Loaded {len(documents)} files")
            debug_print(f"[RAG] Creating vector embeddings with chunking...")

            # Use larger chunk size to keep calendar days together
            text_splitter = SentenceSplitter(
                chunk_size=512,
                chunk_overlap=100
            )

            # Create new index with chunking
            self.index = VectorStoreIndex.from_documents(
                documents,
                storage_context=storage_context,
                transformations=[text_splitter],
                show_progress=True
            )

            # Update retriever
            self.retriever = self.index.as_retriever(similarity_top_k=5)

            debug_print(f"[RAG] Reload complete")
            return True

        except Exception as e:
            debug_print(f"[RAG] Error reloading: {e}")
            import traceback
            traceback.print_exc()
            return False


class ConversationState(Enum):
    """State machine states"""
    LISTENING = 1
    RESPONDING = 2
    SPEAKING = 3


class AudioMode(Enum):
    """Audio input/output mode"""
    LOCAL = 1  # Use local microphone and speakers (default)
    WEB = 2    # Use web browser audio (remote device)


class RosieConversation:
    """
    Main ROSIE Conversational AI System

    Simplified architecture with plain text conversation history.
    """

    def __init__(self, test_mode=False, test_input=None, text_only_mode=False, test_questions_mode=False, test_knowledge_mode=False, test_calendar_mode=False, test_conversation_mode=False):
        """Initialize ROSIE system with configuration from environment

        Args:
            test_mode: If True, run full audio pipeline test (Piper → Speakers → Microphone → Whisper)
            test_input: In test mode, the text to inject (None for interactive)
            text_only_mode: If True, bypass all audio (no Whisper/Piper loading), text I/O only
            test_questions_mode: If True, run comprehension test with paragraphs and questions
            test_knowledge_mode: If True, run RAG knowledge base retrieval test
            test_calendar_mode: If True, run Google Calendar accuracy test
            test_conversation_mode: If True, run conversation quality test with intent classification
        """
        # Test mode configuration
        self.test_mode = test_mode
        self.test_input = test_input
        self.text_only_mode = text_only_mode
        self.test_questions_mode = test_questions_mode
        self.test_knowledge_mode = test_knowledge_mode
        self.test_calendar_mode = test_calendar_mode
        self.test_conversation_mode = test_conversation_mode

        # Load configuration from environment
        self.whisper_model_name = os.getenv('WHISPER_MODEL', 'base')
        self.whisper_chunk_duration = int(os.getenv('WHISPER_CHUNK_DURATION', '3'))

        self.ollama_model = os.getenv('OLLAMA_MODEL', 'llama3.1:8b')
        self.ollama_temperature = float(os.getenv('OLLAMA_TEMPERATURE', '0.7'))  # Default (overridden dynamically)
        self.ollama_max_tokens = int(os.getenv('OLLAMA_MAX_TOKENS', '350'))  # Allow deeper, more substantive responses
        self.ollama_url = 'http://localhost:11434/api/generate'
        self.context_limit = int(os.getenv('CONTEXT_LIMIT', '6000'))  # Token limit before summarization
        #self.context_limit = int(os.getenv('CONTEXT_LIMIT', '1000'))  # Token limit before summarization

        # Piper TTS configuration (from .bashrc)
        self.piper_model_path = os.getenv('PIPER_MODEL_PATH')
        self.piper_config_path = os.getenv('PIPER_CONFIG_PATH')

        # File paths (relative to rosie root directory - parent of src/)
        self.rosie_dir = Path(__file__).parent.parent
        self.history_file = Path(os.getenv('HISTORY_FILE', self.rosie_dir / 'data' / 'conversation_history.txt'))
        self.speak_file = Path(os.getenv('SPEAK_FILE', self.rosie_dir / 'data' / 'speak.txt'))
        self.state_file = Path(os.getenv('STATE_FILE', self.rosie_dir / 'data' / 'rosie_state.json'))
        self.private_mode_change_file = self.rosie_dir / 'data' / 'private_mode_change.json'

        # RAG knowledge base configuration
        self.knowledge_base_dir = Path(os.getenv('KNOWLEDGE_BASE_DIR', self.rosie_dir / 'knowledge_base'))
        self.chroma_db_dir = Path(os.getenv('CHROMA_DB_DIR', self.rosie_dir / 'data' / '.rosie_vector_db'))
        self.rag_system = None  # Initialized later if RAG is available

        # Private mode state (always starts disabled per requirements)
        self.private_mode_enabled = False
        self.private_mode_authenticated = False
        self.last_activity_timestamp = time.time()
        self.private_mode_lock = threading.Lock()

        # Conversation settings
        self.debug = int(os.getenv('DEBUG', '0')) == 1

        # State machine
        self.state = ConversationState.LISTENING
        self.state_lock = threading.Lock()

        # Shutdown event for graceful termination
        self.shutdown_event = threading.Event()

        # Threads
        self.whisper_thread = None
        self.wake_word_thread = None
        self.keyboard_thread = None

        # Audio configuration for continuous streaming
        self.sample_rate = 16000

        # Audio mode (LOCAL or WEB)
        self.audio_mode = AudioMode.LOCAL
        self.audio_mode_lock = threading.Lock()

        # Continuous audio buffer - collects audio while Whisper processes
        self.audio_queue = []  # List of numpy arrays (chunks)
        self.audio_lock = threading.Lock()
        self.audio_stream = None  # sounddevice InputStream

        # Web audio integration
        self.web_server_module = None  # rosie_web_status module (dynamically imported)

        # Whisper model (loaded lazily)
        self.whisper_model = None

        # Wake word detection flag (set by Whisper when wake word detected)
        self.wake_word_detected = False
        self.wake_word_lock = threading.Lock()

        # Demo mode flag (when True, responds to any speech without wake word)
        self.demo_mode = False
        self.demo_mode_lock = threading.Lock()

        # Test mode transcription capture (for audio pipeline testing)
        self.test_last_transcription = None
        self.test_transcription_event = threading.Event()  # Signals transcription complete
        self.test_transcription_lock = threading.Lock()
        self.test_force_transcribe = threading.Event()  # Force immediate transcription

        # TTS interrupt detection (stop speaking when human talks)
        self.tts_interrupt_event = threading.Event()  # Signals interrupt request
        self.tts_process = None  # Current TTS subprocess (Piper + aplay)
        self.tts_process_lock = threading.Lock()  # Protects TTS process access

        # Conversation depth tracking (for transition display)
        self.last_conversation_depth = None

        # Determine total initialization steps based on mode
        # Text-only/test modes: 4 steps (files, calendar, RAG, config)
        # Normal mode: 5 steps (files, calendar, RAG, config, Whisper - loaded in start())
        self._is_text_mode = text_only_mode or test_questions_mode or test_knowledge_mode or test_calendar_mode or test_conversation_mode
        self._total_init_steps = 4 if self._is_text_mode else 5
        self._current_init_step = 0

        # Initialize files
        self._current_init_step += 1
        print(f"(Initialization step {self._current_init_step} of {self._total_init_steps}) Setting up files...", flush=True)
        self._initialize_files()

        # Sync Google Calendar (before RAG so knowledge base has fresh data)
        self._current_init_step += 1
        print(f"(Initialization step {self._current_init_step} of {self._total_init_steps}) Syncing calendar...", flush=True)
        sync_google_calendar(silent=False)

        # Initialize RAG system if available
        self._current_init_step += 1
        print(f"(Initialization step {self._current_init_step} of {self._total_init_steps}) Loading knowledge base...", flush=True)
        self._initialize_rag()

        # Validate configuration
        self._current_init_step += 1
        print(f"(Initialization step {self._current_init_step} of {self._total_init_steps}) Validating configuration...", flush=True)
        self._validate_configuration()

        self._log("ROSIE Conversational AI System initialized")

    def _log(self, message):
        """Log messages with timestamp"""
        if self.debug:
            debug_print(f"[{time.strftime('%H:%M:%S')}] {message}")

    def _initialize_files(self):
        """Initialize conversation files - ensure they exist"""
        # Always clear conversation history on startup for fresh conversations
        self.history_file.write_text('')
        debug_print("Cleared conversation_history.txt (starting fresh)")

        # Always clear speak file (temporary buffer)
        self.speak_file.write_text('')

    def _initialize_rag(self):
        """Initialize RAG knowledge base system"""
        if not RAG_AVAILABLE:
            debug_print("RAG system not available (missing dependencies)")
            return

        try:
            # Always start with private mode OFF
            self.rag_system = RosieRAG(
                knowledge_base_dir=self.knowledge_base_dir,
                chroma_db_dir=self.chroma_db_dir,
                private_mode=False
            )
            debug_print("RAG knowledge base initialized successfully")
        except Exception as e:
            debug_print(f"Warning: Could not initialize RAG system: {e}")
            debug_print("ROSIE will continue without knowledge base features")
            self.rag_system = None

    def _estimate_token_count(self, text):
        """Estimate token count (rough approximation: 1 token ≈ 0.75 words)"""
        words = len(text.split())
        return int(words * 1.3)  # Conservative estimate

    def _validate_configuration(self):
        """Validate required configuration"""
        if not self.piper_model_path or not Path(self.piper_model_path).exists():
            debug_print(f"ERROR: PIPER_MODEL_PATH not found: {self.piper_model_path}")
            debug_print("Please set PIPER_MODEL_PATH in ~/.bashrc")
            sys.exit(1)

    def _get_state(self):
        """Thread-safe state getter"""
        with self.state_lock:
            return self.state

    def _set_state(self, new_state):
        """Thread-safe state setter with web status update"""
        with self.state_lock:
            old_state = self.state
            self.state = new_state
            # Always print state transitions to console
            debug_print(f"\n[STATE] {old_state.name} → {new_state.name}")
            self._log(f"State transition: {old_state.name} -> {new_state.name}")

            # Update web status file
            self._update_web_status(new_state)

    def _get_audio_mode(self):
        """Thread-safe audio mode getter"""
        with self.audio_mode_lock:
            return self.audio_mode

    def _set_audio_mode(self, new_mode):
        """Thread-safe audio mode setter"""
        with self.audio_mode_lock:
            old_mode = self.audio_mode
            self.audio_mode = new_mode
            debug_print(f"\n[AUDIO MODE] {old_mode.name} → {new_mode.name}")
            self._log(f"Audio mode transition: {old_mode.name} -> {new_mode.name}")

    def _check_web_audio_enabled(self):
        """Check if web audio is currently enabled"""
        if self.web_server_module is None:
            # Debug once
            if not hasattr(self, '_web_module_none_warned'):
                debug_print(f"[WEB AUDIO CHECK] web_server_module is None - cannot check status")
                self._web_module_none_warned = True
            return False
        try:
            enabled = self.web_server_module.is_web_audio_enabled()
            # Debug: Show status check results (first 10 times)
            if not hasattr(self, '_web_status_check_count'):
                self._web_status_check_count = 0
            if self._web_status_check_count < 10:
                debug_print(f"[WEB AUDIO CHECK] #{self._web_status_check_count}: enabled={enabled}")
                self._web_status_check_count += 1
            return enabled
        except Exception as e:
            self._log(f"Error checking web audio status: {e}")
            debug_print(f"[WEB AUDIO CHECK] Error: {e}")
            return False

    def _disable_web_audio_status(self):
        """Disable web audio status (clear status file) - for LOCAL mode only"""
        try:
            web_audio_status_file = self.rosie_dir / 'data' / 'web_audio_status.json'
            if web_audio_status_file.exists():
                import json
                with open(web_audio_status_file, 'w') as f:
                    json.dump({'enabled': False, 'client_count': 0, 'timestamp': 0}, f)
                self._log("Web audio status disabled (LOCAL mode)")
        except Exception as e:
            self._log(f"Error disabling web audio status: {e}")

    def _import_web_server_module(self):
        """Dynamically import the web server module for audio integration"""
        if self.web_server_module is not None:
            debug_print("[DEBUG] Web server module already loaded", flush=True)
            return True

        try:
            debug_print("[DEBUG] Attempting to import rosie_web_status module...", flush=True)
            import sys
            from pathlib import Path
            # Add rosie/src to path
            rosie_src_dir = Path(__file__).parent
            debug_print(f"[DEBUG] Adding to sys.path: {rosie_src_dir}", flush=True)
            if str(rosie_src_dir) not in sys.path:
                sys.path.insert(0, str(rosie_src_dir))

            import rosie_web_status
            self.web_server_module = rosie_web_status
            debug_print("[DEBUG] ✓ rosie_web_status module imported successfully!", flush=True)
            self._log("Web server module imported successfully")
            return True
        except ImportError as e:
            debug_print(f"[DEBUG] ✗ Failed to import rosie_web_status: {e}", flush=True)
            self._log(f"Could not import web server module: {e}")
            return False

    def _update_web_status(self, state, custom_message=None):
        """Update web status JSON file for real-time GIF display"""
        try:
            # Map state to GIF filename and message
            state_map = {
                ConversationState.LISTENING: {
                    'state': 'waiting',
                    'gif': 'waiting.gif',
                    'message': 'Waiting for voice input...'
                },
                ConversationState.RESPONDING: {
                    'state': 'thinking',
                    'gif': 'thinking.gif',
                    'message': 'Processing your request...'
                },
                ConversationState.SPEAKING: {
                    'state': 'speaking',
                    'gif': 'speaking.gif',
                    'message': 'Speaking...'
                }
            }

            status = state_map.get(state, state_map[ConversationState.LISTENING])

            # Allow custom message override
            if custom_message:
                status['message'] = custom_message

            # Add private mode fields
            with self.private_mode_lock:
                status['private_mode_enabled'] = self.private_mode_enabled
                status['private_mode_authenticated'] = self.private_mode_authenticated
                status['last_activity_timestamp'] = self.last_activity_timestamp

            # Write to JSON file
            with open(self.state_file, 'w') as f:
                json.dump(status, f, indent=2)

        except Exception as e:
            # Don't let web status updates crash the main system
            self._log(f"Warning: Could not update web status: {e}")

    def _check_private_timeout(self):
        """
        Check if private mode should timeout due to inactivity (30 minutes)

        Returns True if timeout occurred and private mode was disabled
        """
        with self.private_mode_lock:
            # Only check if private mode is enabled and authenticated
            if not (self.private_mode_enabled and self.private_mode_authenticated):
                return False

            # Check if 30 minutes have passed since last activity
            current_time = time.time()
            time_elapsed = current_time - self.last_activity_timestamp
            timeout_seconds = 30 * 60  # 30 minutes

            if time_elapsed >= timeout_seconds:
                debug_print(f"\n[PRIVATE MODE] Timeout after {time_elapsed/60:.1f} minutes of inactivity")
                debug_print(f"[PRIVATE MODE] Disabling private mode...")

                # Disable private mode
                self.private_mode_enabled = False
                self.private_mode_authenticated = False

                # Reload RAG without private documents
                if self.rag_system:
                    success = self.rag_system.reload_with_private_mode(False)
                    if success:
                        debug_print(f"[PRIVATE MODE] RAG reloaded without private documents")
                    else:
                        debug_print(f"[PRIVATE MODE] Warning: RAG reload failed")

                # Update web status
                self._update_web_status(self._get_state())

                return True

            return False

    # =====================================================================
    # STATE 1: LISTENING - Whisper STT and Wake Word Detection
    # =====================================================================

    def _is_hallucination(self, text):
        """
        Detect common Whisper hallucinations

        Returns True if text appears to be a hallucination
        """
        hallucination_patterns = [
            r'subscribe',
            r'youtube',
            r'channel',
            r'like.*comment',
            r'bell icon',
            r'thank(s| you) for watching',
            r'幕',  # Chinese characters (common in training data)
            r'字幕',
            r'살아',  # Korean characters
            r'CC',
            r'\[.*?\]',  # Brackets like [Music], [Applause]
            r'♪',  # Music notes
            r'www\.',
            r'\.com',
            r'http',
            # Voice assistant / AI training data leakage
            r'(piper|whisper).*for.*speech',
            r'speech.*synthesis',
            r'speech.*recognition',
            r'voice assistant',
            r'language model',
            r'artificial intelligence',
            r'she\s+(is|uses|has).*assistant',
            r'transcri(be|bing|ption)',
            r'subtitles? (by|provided|created)',
            # Podcast/video intros
            r'welcome (back )?to',
            r'in this (video|episode)',
            r'today we',
            r"let's (get )?start",
            r'don\'t forget to',
            # Common background noise misinterpretations
            r'^(hmm+|uh+|um+|ah+)[,.\s]*$',
            r'^(yeah|yep|okay|ok)[,.\s]*$',
            r'^it\'?s? good\.?$',
        ]

        text_lower = text.lower()

        # Check for hallucination patterns
        for pattern in hallucination_patterns:
            if re.search(pattern, text_lower):
                return True

        # NEW: Detect excessive repetition (hallucination symptom)
        # Split into words and check for repeated sequences
        words = text.split()
        if len(words) > 10:
            # Check for same word repeated 4+ times in a row
            for i in range(len(words) - 3):
                if words[i] == words[i+1] == words[i+2] == words[i+3]:
                    debug_print(f"[HALLUCINATION] Detected 4x repetition: '{words[i]}'")
                    return True

            # Check for same 2-word phrase repeated 3+ times
            for i in range(len(words) - 5):
                phrase = f"{words[i]} {words[i+1]}"
                phrase2 = f"{words[i+2]} {words[i+3]}"
                phrase3 = f"{words[i+4]} {words[i+5]}"
                if phrase == phrase2 == phrase3:
                    debug_print(f"[HALLUCINATION] Detected 3x phrase repetition: '{phrase}'")
                    return True

        # Check for very short transcriptions (single punctuation or short nonsense)
        if len(text.strip()) <= 2:
            return True

        # Check for repeated punctuation (often hallucination)
        if re.search(r'[.!?]{3,}', text):
            return True

        # Check for gibberish - high ratio of non-ASCII or mixed scripts
        non_ascii_count = sum(1 for c in text if ord(c) > 127)
        if len(text) > 0 and (non_ascii_count / len(text)) > 0.3:
            return True

        # Check for very low ratio of common English words
        common_words = ['the', 'is', 'are', 'was', 'were', 'a', 'an', 'to', 'of', 'in', 'on', 'for', 'with', 'you', 'i', 'me', 'my', 'your']
        words = text_lower.split()
        if len(words) > 5:  # Only check longer phrases
            common_word_count = sum(1 for word in words if word in common_words)
            if common_word_count == 0:
                return True

        return False

    def _load_whisper_model(self):
        """Load faster-whisper model with GPU auto-detection and superior hallucination suppression"""
        debug_print("[LOAD MODEL] Checking if model already loaded...", flush=True)
        if self.whisper_model is None:
            debug_print("[LOAD MODEL] Model not loaded, starting load process...", flush=True)
            self._log(f"Loading faster-whisper model: {self.whisper_model_name}")

            try:
                # Suppress faster-whisper's INFO logging (VAD messages) unless in debug mode
                if not DEBUG_MODE:
                    import logging
                    logging.getLogger("faster_whisper").setLevel(logging.WARNING)

                debug_print("[LOAD MODEL] Importing WhisperModel...", flush=True)
                from faster_whisper import WhisperModel

                # Auto-detect best available device
                debug_print("[LOAD MODEL] Detecting best device...", flush=True)
                device = self._detect_best_device()

                # Use float16 with CUDA (now that cuDNN is installed), int8 for CPU
                compute_type = "float16" if device == "cuda" else "int8"

                debug_print(f"[LOAD MODEL] Using device: {device}, compute_type: {compute_type}", flush=True)
                self._log(f"Using device: {device}, compute_type: {compute_type}")

                # Load faster-whisper model with optimizations
                debug_print(f"[LOAD MODEL] Creating WhisperModel instance (THIS MAY TAKE 10-30 SECONDS)...", flush=True)
                self.whisper_model = WhisperModel(
                    self.whisper_model_name,
                    device=device,
                    compute_type=compute_type,
                    num_workers=1  # Single worker for real-time
                )
                debug_print("[LOAD MODEL] WhisperModel instance created", flush=True)

                if device == "cuda":
                    debug_print(f"[WHISPER] ✓ Faster-Whisper loaded on GPU (CUDA, {compute_type}) - 4-6x faster!")
                else:
                    debug_print(f"[WHISPER] ℹ Faster-Whisper loaded on CPU")

                self._log(f"Faster-Whisper model loaded successfully on {device}")

            except Exception as e:
                debug_print(f"\n[ERROR] Failed to load Faster-Whisper model: {e}")
                debug_print(f"[ERROR] Please check faster-whisper installation and cuDNN")
                self._log(f"Faster-Whisper model loading failed: {e}")
                raise

    def _detect_best_device(self):
        """Detect best available device for Whisper (GPU with fallback to CPU)"""
        try:
            import torch
            import warnings

            # Suppress CUDA initialization warnings (system-level PyTorch/CUDA mismatch)
            with warnings.catch_warnings():
                warnings.filterwarnings("ignore", category=UserWarning, message=".*CUDA initialization.*")

                # Check if CUDA is available and working
                if torch.cuda.is_available():
                    try:
                        # Test if CUDA actually works by creating a small tensor
                        test_tensor = torch.zeros(1).cuda()
                        del test_tensor
                        self._log("CUDA detected and functional")
                        return "cuda"
                    except Exception as e:
                        self._log(f"CUDA available but not functional: {e}")
                        debug_print(f"[WHISPER] ⚠ GPU detected but CUDA initialization failed")
                        debug_print(f"[WHISPER] → Falling back to CPU")
                        return "cpu"
                else:
                    self._log("CUDA not available, using CPU")
                    return "cpu"

        except Exception as e:
            self._log(f"Device detection error: {e}, defaulting to CPU")
            return "cpu"

    def _audio_callback(self, indata, frames, time_info, status):
        """
        Audio input callback - continuously captures audio from LOCAL microphone

        This runs in a separate thread managed by sounddevice.
        Captures ALL audio without gaps.
        Only used in LOCAL audio mode.
        """
        if status:
            self._log(f"Audio callback status: {status}")
            debug_print(f"[AUDIO] Callback status: {status}")

        # Only capture during LISTENING state and LOCAL audio mode
        current_mode = self._get_audio_mode()
        current_state = self._get_state()

        # Debug: Print first few callbacks to verify audio stream is working
        if not hasattr(self, '_callback_count'):
            self._callback_count = 0
        if self._callback_count < 3:
            debug_print(f"[AUDIO CALLBACK] #{self._callback_count}: state={current_state}, mode={current_mode}, frames={frames}, max_level={np.abs(indata).max():.4f}")
            self._callback_count += 1

        if current_state == ConversationState.LISTENING and current_mode == AudioMode.LOCAL:
            # Append audio to queue (thread-safe)
            with self.audio_lock:
                self.audio_queue.append(indata.copy().flatten())

    def _web_audio_poll_worker(self):
        """
        Background worker that polls web server for audio input from browser.
        Only runs when web audio mode is active.
        Adds received audio to the same queue that _audio_callback uses.
        """
        self._log("Web audio poll worker started")

        while not self.shutdown_event.is_set():
            try:
                # Check if web audio is enabled
                web_audio_enabled = self._check_web_audio_enabled()

                if web_audio_enabled:
                    # Switch to WEB mode if not already
                    if self._get_audio_mode() != AudioMode.WEB:
                        debug_print(f"[WEB AUDIO] Detected web audio enabled, switching to WEB mode")
                        self._set_audio_mode(AudioMode.WEB)

                    # Poll for audio data from web server
                    if self.web_server_module:
                        audio_data = self.web_server_module.get_web_audio_input()
                        if audio_data is not None:
                            # Convert Int16Array buffer to numpy array
                            audio_array = np.frombuffer(audio_data, dtype=np.int16)
                            # Convert to float32 [-1.0, 1.0] format (same as sounddevice)
                            audio_float = audio_array.astype(np.float32) / 32768.0

                            # Add to audio queue (same as _audio_callback)
                            with self.audio_lock:
                                self.audio_queue.append(audio_float)
                    else:
                        # Debug: Show if module not available
                        if not hasattr(self, '_web_module_warning_shown'):
                            debug_print(f"[WEB AUDIO] Warning: web_server_module not available")
                            self._web_module_warning_shown = True

                else:
                    # Switch back to LOCAL mode if not already
                    if self._get_audio_mode() != AudioMode.LOCAL:
                        debug_print(f"[WEB AUDIO] Web audio disabled, switching back to LOCAL mode")
                        self._set_audio_mode(AudioMode.LOCAL)

                # Poll frequency: 25ms when active (40 polls/sec), 100ms when inactive
                # Browser sends audio every ~256ms, so 25ms polling is plenty responsive
                if self._get_audio_mode() == AudioMode.WEB:
                    time.sleep(0.025)  # 25ms - poll 40 times/sec (was 5ms/200x which was overkill)
                else:
                    time.sleep(0.1)  # 100ms - slower polling when not in use

            except Exception as e:
                self._log(f"Error in web audio poll worker: {e}")
                time.sleep(0.1)

        self._log("Web audio poll worker stopped")

    def _whisper_worker(self):
        """
        Whisper speech-to-text worker thread with Voice Activity Detection

        Uses VAD to detect when user finishes speaking, then transcribes complete phrases.
        NO MORE PARTIAL PHRASES!
        """
        self._log("Whisper worker started")
        # Note: Whisper model is already loaded in start() before this thread starts

        # Import VAD
        import webrtcvad
        # Start with mode 3 (most aggressive) for web audio compatibility
        # Will dynamically adjust based on audio mode
        vad = webrtcvad.Vad(2)  # Aggressiveness: 0-3 (2 = balanced, less likely to cut off speech)

        # Start continuous audio stream
        self._start_audio_stream()

        # VAD configuration
        vad_frame_duration = 30  # ms per frame (must be 10, 20, or 30)
        vad_frames_per_check = int(self.sample_rate * vad_frame_duration / 1000)
        speech_buffer = []  # Accumulates audio while speaking
        consecutive_silence = 0
        is_speaking = False

        # Buffer limit cooldown (prevents infinite loop with web audio background noise)
        buffer_limit_cooldown_until = 0  # Timestamp when cooldown expires

        while not self.shutdown_event.is_set():
            current_state = self._get_state()

            # Only process in LISTENING state
            if current_state == ConversationState.LISTENING:
                try:
                    # Debug: Show mode at beginning of processing
                    if not hasattr(self, '_mode_check_count'):
                        self._mode_check_count = 0
                    if self._mode_check_count < 5:
                        current_mode = self._get_audio_mode()
                        queue_len = len(self.audio_queue)
                        debug_print(f"[WHISPER WORKER] Mode: {current_mode}, Queue length: {queue_len}")
                        self._mode_check_count += 1

                    # Check audio queue every 30ms
                    time.sleep(vad_frame_duration / 1000.0)

                    # Get accumulated audio from queue
                    audio_from_queue = False
                    with self.audio_lock:
                        queue_len = len(self.audio_queue)
                        if queue_len == 0:
                            # No new audio - treat as silence frame for VAD checking only
                            # Critical for WEB mode where chunks arrive less frequently
                            audio_chunk = np.zeros(vad_frames_per_check, dtype=np.float32)
                        else:
                            # Get real audio from queue
                            audio_chunk = self.audio_queue.pop(0)
                            audio_from_queue = True

                    # Convert to int16 for VAD
                    audio_int16 = (audio_chunk * 32767).astype(np.int16)

                    # Pad or trim to exact frame size
                    if len(audio_int16) < vad_frames_per_check:
                        audio_int16 = np.pad(audio_int16, (0, vad_frames_per_check - len(audio_int16)))
                    elif len(audio_int16) > vad_frames_per_check:
                        audio_int16 = audio_int16[:vad_frames_per_check]

                    # Pre-filter very quiet audio (background noise) before VAD - WEB mode only
                    # This helps WEB mode where browser sends continuous chunks with background noise
                    # that confuses VAD and prevents silence detection
                    if self._get_audio_mode() == AudioMode.WEB:
                        # Safety check for empty audio chunks
                        if len(audio_chunk) == 0:
                            is_speech = False
                        else:
                            # Multi-layer filtering for web audio noise rejection
                            # Layer 1: RMS energy (average power across entire chunk)
                            rms_energy = np.sqrt(np.mean(audio_chunk ** 2))

                            # Layer 2: Peak level (max amplitude)
                            peak_level = np.abs(audio_chunk).max()

                            # Layer 3: Audio variation (std dev - speech varies, noise is flat)
                            audio_variation = np.std(audio_chunk)

                            # Debug: Show metrics for first few web audio checks
                            if not hasattr(self, '_web_vad_check_count'):
                                self._web_vad_check_count = 0
                            if self._web_vad_check_count < 5:
                                debug_print(f"[WEB VAD] #{self._web_vad_check_count}: rms={rms_energy:.4f}, peak={peak_level:.4f}, var={audio_variation:.4f}")
                                self._web_vad_check_count += 1

                            # Require ALL of: sufficient RMS, sufficient peak, AND variation
                            # This rejects constant background noise while accepting real speech
                            # Web audio needs more aggressive filtering to prevent false triggers
                            if rms_energy < 0.02 or peak_level < 0.08 or audio_variation < 0.015:
                                is_speech = False  # Noise rejected
                            else:
                                # Passed pre-filters - check VAD
                                try:
                                    is_speech = vad.is_speech(audio_int16.tobytes(), self.sample_rate)
                                except:
                                    continue
                    else:
                        # LOCAL mode - use VAD directly (no pre-filtering needed)
                        try:
                            # Debug: Show VAD attempt
                            if not hasattr(self, '_vad_attempt_count'):
                                self._vad_attempt_count = 0
                            if self._vad_attempt_count < 5:
                                audio_level = np.abs(audio_chunk).max() if len(audio_chunk) > 0 else 0
                                debug_print(f"[VAD ATTEMPT] #{self._vad_attempt_count}: audio_level={audio_level:.4f}, chunk_len={len(audio_chunk)}, int16_len={len(audio_int16)}, from_queue={audio_from_queue}")
                                self._vad_attempt_count += 1

                            is_speech = vad.is_speech(audio_int16.tobytes(), self.sample_rate)

                            # Debug: Show VAD results for first few checks
                            if not hasattr(self, '_vad_check_count'):
                                self._vad_check_count = 0
                            if self._vad_check_count < 10:
                                debug_print(f"[VAD CHECK] #{self._vad_check_count}: is_speech={is_speech}")
                                self._vad_check_count += 1
                        except Exception as e:
                            if not hasattr(self, '_vad_error_count'):
                                self._vad_error_count = 0
                            if self._vad_error_count < 5:
                                debug_print(f"[VAD ERROR] #{self._vad_error_count}: {e}, chunk_len={len(audio_chunk)}, int16_len={len(audio_int16)}")
                                self._vad_error_count += 1
                            continue

                    # Check buffer limit cooldown (prevents false trigger loops with web audio)
                    if time.time() < buffer_limit_cooldown_until:
                        is_speech = False  # Ignore all speech during cooldown period
                        # Clear audio queue during cooldown to prevent buildup
                        with self.audio_lock:
                            if len(self.audio_queue) > 0:
                                self.audio_queue.clear()
                        # Debug: Show cooldown status (only first few times)
                        if not hasattr(self, '_cooldown_msg_count'):
                            self._cooldown_msg_count = 0
                        if self._cooldown_msg_count < 3:
                            remaining = buffer_limit_cooldown_until - time.time()
                            debug_print(f"[COOLDOWN] Ignoring audio for {remaining:.1f}s more...")
                            self._cooldown_msg_count += 1

                    if is_speech:
                        # Voice detected!
                        if not is_speaking:
                            # First detection of speech - START of new phrase
                            is_speaking = True
                            consecutive_silence = 0  # Reset counter for new phrase
                            debug_print(f"[SPEECH START] is_speaking set to True, resetting consecutive_silence")
                        else:
                            # Continuing speech - do NOT reset counter
                            # Allow brief silence during speech to accumulate toward threshold
                            pass
                        if audio_from_queue:
                            speech_buffer.append(audio_chunk)
                            # Debug: Show when speech is detected
                            if len(speech_buffer) <= 3:
                                debug_print(f"[VAD] Speech detected! Buffer size: {len(speech_buffer)}")

                            # NEW: Prevent buffer overflow - limit audio duration
                            # Each chunk is ~100ms, so 30 chunks = 3 seconds
                            # Test mode: no limit (uses sys.maxsize) to capture any length
                            MAX_BUFFER_CHUNKS = (2**31 - 1) if self.test_mode else 30
                            if len(speech_buffer) > MAX_BUFFER_CHUNKS:
                                debug_print(f"[BUFFER LIMIT] Reached max buffer size ({MAX_BUFFER_CHUNKS} chunks), forcing transcription")
                                # Force transcription by simulating silence threshold
                                consecutive_silence = 999
                                # Activate cooldown to prevent immediate re-trigger (web audio continuous stream issue)
                                buffer_limit_cooldown_until = time.time() + 2.0  # 2 second cooldown
                                debug_print(f"[BUFFER LIMIT] Activating 2-second cooldown to prevent loop")
                                # Reset cooldown message counter for new cooldown period
                                self._cooldown_msg_count = 0
                    elif is_speaking:
                        # Silence while speaking - might be end of phrase
                        consecutive_silence += 1
                        # Only add real audio to buffer, not generated silence frames
                        if audio_from_queue:
                            speech_buffer.append(audio_chunk)

                        # Check if enough silence to consider phrase complete
                        # Test mode: 5 seconds (long paragraphs have natural pauses)
                        # Demo mode: 2 seconds, Normal mode: 0.5 seconds
                        with self.demo_mode_lock:
                            in_demo_mode = self.demo_mode
                        if self.test_mode:
                            silence_threshold = 5.0
                        elif in_demo_mode:
                            silence_threshold = 2.0
                        else:
                            silence_threshold = 0.5
                        silence_frames_needed = int(silence_threshold * 1000 / vad_frame_duration)

                        # Check for test mode force transcribe signal
                        force_transcribe = self.test_mode and self.test_force_transcribe.is_set()

                        # In test mode, ONLY transcribe when force_transcribe is set (ignore silence threshold)
                        # This ensures we capture the full audio before transcribing
                        should_transcribe = force_transcribe if self.test_mode else (consecutive_silence >= silence_frames_needed)

                        if should_transcribe:
                            # Phrase complete! Transcribe it
                            if force_transcribe:
                                debug_print(f"[TEST] Force transcribe triggered")
                            debug_print(f"[TRANSCRIBE] Silence threshold reached! consecutive_silence={consecutive_silence}, threshold={silence_frames_needed}")
                            if len(speech_buffer) > 0:
                                audio_data = np.concatenate(speech_buffer)
                                debug_print(f"[TRANSCRIBE] Concatenated {len(speech_buffer)} chunks into {len(audio_data)} samples")

                                # Check if there's actual speech (more aggressive filtering)
                                audio_level = np.abs(audio_data).max()
                                rms_level = np.sqrt(np.mean(audio_data ** 2))
                                debug_print(f"[TRANSCRIBE] Audio level check: peak={audio_level:.4f}, rms={rms_level:.4f}")
                                # Require both peak and RMS to be above thresholds
                                if audio_level > 0.05 and rms_level > 0.01:
                                    debug_print(f"[TRANSCRIBE] Starting Whisper transcription...")
                                    # Transcribe with faster-whisper (superior hallucination suppression)
                                    segments, info = self.whisper_model.transcribe(
                                        audio_data,
                                        language='en',
                                        beam_size=5,
                                        best_of=5,
                                        temperature=0.0,
                                        condition_on_previous_text=False,
                                        # NOTE: Do NOT use initial_prompt - Whisper often outputs the prompt
                                        # itself as hallucination when audio is unclear
                                        vad_filter=True,  # Extra VAD filtering from faster-whisper
                                        vad_parameters=dict(min_silence_duration_ms=500),
                                        # CRITICAL: Skip segments with >50% silence to prevent hallucinations
                                        hallucination_silence_threshold=0.5,  # Skip if >50% silence
                                        # Additional anti-hallucination measures
                                        compression_ratio_threshold=2.4,  # Default, but explicit
                                        log_prob_threshold=-1.0,  # Skip low-confidence segments
                                        no_speech_threshold=0.6  # Higher threshold for "no speech" detection
                                    )

                                    # Extract text from segments generator
                                    transcription = ' '.join([segment.text for segment in segments]).strip()
                                    debug_print(f"[TRANSCRIBE] Whisper returned: '{transcription}'")

                                    # In test mode, capture transcription for validation
                                    if self.test_mode:
                                        with self.test_transcription_lock:
                                            self.test_last_transcription = transcription
                                            self.test_transcription_event.set()  # Signal transcription ready
                                        debug_print(f"[TEST] Captured transcription: '{transcription}'")

                                    # Filter out common Whisper hallucinations
                                    if transcription and not self._is_hallucination(transcription):
                                        debug_print(f"[TRANSCRIBE] Passed hallucination filter")
                                        # Check if in demo mode
                                        with self.demo_mode_lock:
                                            in_demo_mode = self.demo_mode

                                        if in_demo_mode:
                                            # Demo mode: Respond to ANY speech without wake word
                                            with self.wake_word_lock:
                                                self.wake_word_detected = True

                                            # Store transcription as-is
                                            self._append_to_history(f"Human: {transcription}\n")
                                            print(f"[Whisper] {transcription}")
                                            debug_print(f"[WHISPER] [DEMO MODE] Human: {transcription}")
                                            self._log(f"Demo mode transcribed: {transcription}")
                                        else:
                                            # Normal mode: Check for wake word BEFORE storing
                                            wake_word_pattern = r'\b(rosie|rose|rosy|rosee)\b'
                                            wake_word_match = re.search(wake_word_pattern, transcription, re.IGNORECASE)

                                            if wake_word_match:
                                                # Wake word detected! Set flag and remove it from transcription
                                                with self.wake_word_lock:
                                                    self.wake_word_detected = True

                                                # Remove wake word from transcription before storing
                                                cleaned_transcription = re.sub(wake_word_pattern, '', transcription, flags=re.IGNORECASE).strip()

                                                print(f"[Whisper] {transcription}")
                                                self._log(f"Wake word detected in: {transcription}")

                                                # Store cleaned version (without wake word)
                                                if cleaned_transcription:
                                                    self._append_to_history(f"Human: {cleaned_transcription}\n")
                                                    # Only show what was said to ROSIE (without wake word)
                                            else:
                                                # No wake word, store as-is (ambient conversation for context)
                                                self._append_to_history(f"Human: {transcription}\n")
                                                print(f"[Whisper] {transcription}")
                                                self._log(f"Transcribed: {transcription}")
                                    elif transcription:
                                        self._log(f"Filtered hallucination: {transcription}")

                            # Reset for next phrase
                            debug_print(f"[TRANSCRIBE] Resetting speech buffer and state")
                            speech_buffer = []
                            consecutive_silence = 0
                            is_speaking = False

                except Exception as e:
                    self._log(f"Whisper VAD error: {e}")
                    import traceback
                    traceback.print_exc()

            elif current_state == ConversationState.SPEAKING:
                # Monitor for human speech during TTS playback to enable interrupts

                # Check audio queue every 30ms
                time.sleep(0.03)

                # Get audio chunk from queue
                audio_from_queue = False
                with self.audio_lock:
                    if len(self.audio_queue) > 0:
                        audio_chunk = self.audio_queue.pop(0)
                        audio_from_queue = True
                    else:
                        continue  # No audio, keep waiting

                # Convert to int16 for VAD
                audio_int16 = (audio_chunk * 32767).astype(np.int16)

                # Pad or trim to VAD frame size (480 samples for 30ms at 16kHz)
                vad_frames_per_check = 480
                if len(audio_int16) < vad_frames_per_check:
                    audio_int16 = np.pad(audio_int16, (0, vad_frames_per_check - len(audio_int16)))
                elif len(audio_int16) > vad_frames_per_check:
                    audio_int16 = audio_int16[:vad_frames_per_check]

                # Pre-filter to reduce TTS echo false positives
                audio_level = np.abs(audio_chunk).max()

                # Higher threshold during SPEAKING to avoid detecting TTS echo
                if audio_level > 0.02:  # Louder than background/echo
                    try:
                        is_speech = vad.is_speech(audio_int16.tobytes(), self.sample_rate)

                        # Require 2 consecutive detections for reliability
                        if is_speech:
                            if not hasattr(self, '_interrupt_speech_count'):
                                self._interrupt_speech_count = 0
                            self._interrupt_speech_count += 1

                            if self._interrupt_speech_count >= 2:  # 60ms total
                                debug_print("[INTERRUPT] Human speech detected during TTS playback")
                                self.tts_interrupt_event.set()
                                self._interrupt_speech_count = 0
                                # Clear audio queue and reset speech tracking
                                with self.audio_lock:
                                    self.audio_queue.clear()
                                speech_buffer = []
                                is_speaking = False
                                consecutive_silence = 0
                                # Transition back to LISTENING
                                self._set_state(ConversationState.LISTENING)
                        else:
                            self._interrupt_speech_count = 0
                    except Exception as e:
                        self._log(f"Interrupt detection error: {e}")
                else:
                    self._interrupt_speech_count = 0

            else:
                # RESPONDING state - clear queue, don't transcribe LLM processing
                with self.audio_lock:
                    self.audio_queue.clear()
                speech_buffer = []
                is_speaking = False
                consecutive_silence = 0
                time.sleep(0.1)

        # Stop audio stream
        self._stop_audio_stream()
        self._log("Whisper worker stopped")

    def _start_audio_stream(self):
        """Start continuous audio capture stream"""
        if self.audio_stream is None:
            self._log("Starting continuous audio stream")

            # Get default input device info for debugging
            try:
                default_device = sd.query_devices(kind='input')
                self._log(f"Using audio device: {default_device['name']} (index {default_device['index']})")
            except Exception as e:
                self._log(f"Error querying audio device: {e}")

            try:
                self.audio_stream = sd.InputStream(
                    samplerate=self.sample_rate,
                    channels=1,
                    dtype='float32',
                    callback=self._audio_callback,
                    blocksize=int(self.sample_rate * 0.1)  # 100ms blocks
                )
                self.audio_stream.start()
                self._log(f"Audio stream started successfully - sample_rate={self.sample_rate}")
                debug_print(f"\n[AUDIO MODE] LOCAL microphone enabled")
            except Exception as e:
                self._log(f"ERROR starting audio stream: {e}")
                debug_print(f"\n[ERROR] Failed to start audio stream: {e}")
                import traceback
                traceback.print_exc()

    def _stop_audio_stream(self):
        """Stop continuous audio capture stream"""
        if self.audio_stream is not None:
            self._log("Stopping audio stream")
            try:
                self.audio_stream.stop()
            except Exception as e:
                self._log(f"Error stopping stream: {e}")

            try:
                self.audio_stream.close()
            except Exception as e:
                self._log(f"Error closing stream: {e}")

            self.audio_stream = None
            self._log("Audio stream stopped")

    def _append_to_history(self, text):
        """Append text to conversation_history.txt with timestamp (thread-safe)"""
        try:
            # Add timestamp to Human: or Robot: lines
            if text.startswith('Human:') or text.startswith('Robot:'):
                # Format: Human [2025-11-8-14:30]: or Robot [2025-11-8-14:30]:
                timestamp = time.strftime('%Y-%m-%d-%H:%M')
                prefix = text.split(':', 1)[0]  # Get "Human" or "Robot"
                rest = text.split(':', 1)[1] if ':' in text else ''
                text = f"{prefix} [{timestamp}]:{rest}"

            with open(self.history_file, 'a') as f:
                f.write(text)
        except Exception as e:
            self._log(f"Error writing to conversation_history.txt: {e}")

    def _calculate_transcription_accuracy(self, original, transcribed):
        """
        Calculate accuracy between original text and transcribed text

        Uses character-level and word-level comparison with normalization

        Args:
            original: The original input text
            transcribed: The transcribed text from Whisper

        Returns:
            tuple: (character_accuracy, word_accuracy) as percentages (0-100)
        """
        import re as regex

        # Normalize both strings for comparison:
        # - lowercase
        # - collapse multiple whitespace to single space
        # - strip leading/trailing whitespace
        orig_normalized = regex.sub(r'\s+', ' ', original.lower().strip())
        trans_normalized = regex.sub(r'\s+', ' ', transcribed.lower().strip())

        # Character-level accuracy using Levenshtein distance
        if len(orig_normalized) == 0:
            char_accuracy = 100.0 if len(trans_normalized) == 0 else 0.0
        else:
            # Calculate Levenshtein distance (edit distance)
            def levenshtein(s1, s2):
                if len(s1) < len(s2):
                    return levenshtein(s2, s1)
                if len(s2) == 0:
                    return len(s1)
                prev_row = range(len(s2) + 1)
                for i, c1 in enumerate(s1):
                    curr_row = [i + 1]
                    for j, c2 in enumerate(s2):
                        insertions = prev_row[j + 1] + 1
                        deletions = curr_row[j] + 1
                        substitutions = prev_row[j] + (c1 != c2)
                        curr_row.append(min(insertions, deletions, substitutions))
                    prev_row = curr_row
                return prev_row[-1]

            distance = levenshtein(orig_normalized, trans_normalized)
            max_len = max(len(orig_normalized), len(trans_normalized))
            char_accuracy = ((max_len - distance) / max_len) * 100.0

        # Word-level accuracy using Levenshtein distance on words
        # Also normalize hyphens (treat hyphenated words as separate)
        orig_for_words = regex.sub(r'-', ' ', orig_normalized)
        trans_for_words = regex.sub(r'-', ' ', trans_normalized)
        orig_words = orig_for_words.split()
        trans_words = trans_for_words.split()

        if len(orig_words) == 0:
            word_accuracy = 100.0 if len(trans_words) == 0 else 0.0
        else:
            # Use Levenshtein distance on word lists
            def word_levenshtein(s1, s2):
                if len(s1) < len(s2):
                    return word_levenshtein(s2, s1)
                if len(s2) == 0:
                    return len(s1)
                prev_row = range(len(s2) + 1)
                for i, w1 in enumerate(s1):
                    curr_row = [i + 1]
                    for j, w2 in enumerate(s2):
                        insertions = prev_row[j + 1] + 1
                        deletions = curr_row[j] + 1
                        substitutions = prev_row[j] + (w1 != w2)
                        curr_row.append(min(insertions, deletions, substitutions))
                    prev_row = curr_row
                return prev_row[-1]

            word_distance = word_levenshtein(orig_words, trans_words)
            max_words = max(len(orig_words), len(trans_words))
            word_accuracy = ((max_words - word_distance) / max_words) * 100.0

        return char_accuracy, word_accuracy

    def _run_test_questions(self):
        """
        Run comprehension test with paragraphs and questions.

        Tests ROSIE's ability to understand context and answer questions accurately.
        Simulates passive listening for the paragraph, then triggers response with wake word.
        No TTS output - text only.
        """
        import random
        import datetime

        # Test cases: each has a paragraph, question (with Rosie), and expected answer keywords
        test_cases = [
            {
                "paragraph": "The Great Wall of China was built over many centuries, with construction beginning as early as the 7th century BC. The most well-known sections were built during the Ming Dynasty between 1368 and 1644. The wall stretches approximately 13,171 miles and was primarily built to protect against invasions from northern nomadic groups.",
                "question": "Rosie, during which dynasty were the most well-known sections of the Great Wall built?",
                "expected_keywords": ["ming", "dynasty"],
                "expected_answer": "Ming Dynasty"
            },
            {
                "paragraph": "Marie Curie was a Polish-born physicist and chemist who conducted pioneering research on radioactivity. She was the first woman to win a Nobel Prize and remains the only person to have won Nobel Prizes in two different sciences: Physics in 1903 and Chemistry in 1911.",
                "question": "Rosie, in what year did Marie Curie win the Nobel Prize in Chemistry?",
                "expected_keywords": ["1911"],
                "expected_answer": "1911"
            },
            {
                "paragraph": "The Amazon River is the largest river by discharge volume of water in the world. It flows through South America, primarily through Brazil, and is approximately 4,000 miles long. The Amazon basin contains the world's largest tropical rainforest, covering about 2.72 million square miles.",
                "question": "Rosie, approximately how long is the Amazon River?",
                "expected_keywords": ["4000", "4,000", "four thousand"],
                "expected_answer": "4,000 miles"
            },
            {
                "paragraph": "The human heart beats approximately 100,000 times per day, pumping about 2,000 gallons of blood through the body. The heart has four chambers: two atria and two ventricles. The average adult heart weighs between 8 and 12 ounces.",
                "question": "Rosie, how many chambers does the human heart have?",
                "expected_keywords": ["four", "4"],
                "expected_answer": "four chambers"
            },
            {
                "paragraph": "Mount Everest, located in the Himalayas on the border between Nepal and Tibet, is the highest mountain on Earth. Its peak stands at 29,032 feet above sea level. The first confirmed summit was achieved by Edmund Hillary and Tenzing Norgay on May 29, 1953.",
                "question": "Rosie, who were the first people to reach the summit of Mount Everest?",
                "expected_keywords": ["hillary", "norgay", "edmund", "tenzing"],
                "expected_answer": "Edmund Hillary and Tenzing Norgay"
            },
            {
                "paragraph": "The speed of light in a vacuum is exactly 299,792,458 meters per second, which is approximately 186,282 miles per second. Light from the Sun takes about 8 minutes and 20 seconds to reach Earth. This constant speed is fundamental to Einstein's theory of special relativity.",
                "question": "Rosie, how long does it take for light from the Sun to reach Earth?",
                "expected_keywords": ["8", "eight", "minute", "20", "twenty", "second"],
                "expected_answer": "8 minutes and 20 seconds"
            },
            {
                "paragraph": "The Eiffel Tower was completed in 1889 and was built as the entrance arch for the World's Fair held in Paris that year. Designed by Gustave Eiffel's engineering company, it stands 1,083 feet tall and was the tallest man-made structure in the world until 1930.",
                "question": "Rosie, in what year was the Eiffel Tower completed?",
                "expected_keywords": ["1889"],
                "expected_answer": "1889"
            },
            {
                "paragraph": "Water freezes at 32 degrees Fahrenheit or 0 degrees Celsius at standard atmospheric pressure. It boils at 212 degrees Fahrenheit or 100 degrees Celsius. Water is unique because it is less dense as a solid than as a liquid, which is why ice floats.",
                "question": "Rosie, at what temperature does water boil in Fahrenheit?",
                "expected_keywords": ["212"],
                "expected_answer": "212 degrees Fahrenheit"
            },
        ]

        # Randomly select a test case
        test_case = random.choice(test_cases)

        print("\n" + "="*70)
        print("ROSIE COMPREHENSION TEST")
        print("="*70)
        print("\n[STEP 1] Simulating passive listening (STT transcription)...\n")
        print(f"Paragraph: {test_case['paragraph']}")
        print()
        print("(ROSIE passively listening - no response)")

        # Step 1: Add paragraph to conversation history as passive listening
        # This simulates what would happen with STT transcription without wake word
        timestamp = datetime.datetime.now().strftime('%I:%M %p')
        history_entry = f"Human [{timestamp}]: {test_case['paragraph']}\n"
        with open(self.history_file, 'a') as f:
            f.write(history_entry)

        # No response from ROSIE - just passive listening

        print("\n" + "-"*70)
        print("\n[STEP 2] Asking question with wake word 'Rosie'...\n")
        print(f"Question: {test_case['question']}")
        print(f"Expected answer: {test_case['expected_answer']}")
        print()

        # Step 2: Ask the question (includes "Rosie" to trigger response)
        timestamp = datetime.datetime.now().strftime('%I:%M %p')
        history_entry = f"Human [{timestamp}]: {test_case['question']}\n"
        with open(self.history_file, 'a') as f:
            f.write(history_entry)

        # Get ROSIE's answer (text only, no TTS)
        self._ollama_response()
        answer_response = self.speak_file.read_text().strip()

        # Add ROSIE's answer to history
        timestamp = datetime.datetime.now().strftime('%I:%M %p')
        history_entry = f"Robot [{timestamp}]: {answer_response}\n"
        with open(self.history_file, 'a') as f:
            f.write(history_entry)

        print(f"ROSIE answer: {answer_response}\n")

        print("-"*70)
        print("\n[STEP 3] Evaluating response accuracy...\n")

        # Check if response contains expected keywords
        # Keywords are treated as alternatives - finding ANY one of them means success
        answer_lower = answer_response.lower()
        keywords_found = []

        for keyword in test_case['expected_keywords']:
            if keyword.lower() in answer_lower:
                keywords_found.append(keyword)

        # Success if at least one keyword variant was found
        passed = len(keywords_found) > 0

        print(f"Expected (any of): {test_case['expected_keywords']}")
        print(f"Found: {keywords_found if keywords_found else 'None'}")

        print("\n" + "="*70)
        if passed:
            print("TEST RESULT: ✓ PASSED - Expected answer found")
            result = True
        else:
            print("TEST RESULT: ✗ FAILED - Expected answer not found")
            result = False
        print("="*70 + "\n")

        return result

    def _run_test_knowledge(self):
        """
        Run RAG knowledge base retrieval test.

        Tests ROSIE's ability to retrieve and use information from the knowledge_base folder.
        Asks a question about data in the non-private documents.
        """
        import random
        import datetime

        # Test cases based on knowledge_base content (non-private files)
        # Each test has a question with "Rosie" and expected keywords from the RAG data
        test_cases = [
            {
                "source": "family.md",
                "question": "Rosie, what are the names of the two dogs in the family?",
                "expected_keywords": ["luke", "henry"],
                "expected_answer": "Luke and Henry"
            },
            {
                "source": "family.md",
                "question": "Rosie, what breed is Henry the dog?",
                "expected_keywords": ["boxer"],
                "expected_answer": "boxer mix"
            },
            {
                "source": "family.md",
                "question": "Rosie, what type of car does Emerson like to work on?",
                "expected_keywords": ["miata", "1995"],
                "expected_answer": "1995 Miata"
            },
            {
                "source": "family.md",
                "question": "Rosie, what university is Annie graduating from?",
                "expected_keywords": ["texas state", "texas"],
                "expected_answer": "Texas State University"
            },
            {
                "source": "family.md",
                "question": "Rosie, what sport do Mike and Emerson play together?",
                "expected_keywords": ["pickleball"],
                "expected_answer": "pickleball"
            },
            {
                "source": "family.md",
                "question": "Rosie, what exercise class do Mike and Michelle take on Saturday mornings?",
                "expected_keywords": ["pilates"],
                "expected_answer": "Pilates"
            },
            {
                "source": "yardcare.md",
                "question": "Rosie, what month should I overseed with ryegrass according to the yard care plan?",
                "expected_keywords": ["november"],
                "expected_answer": "November"
            },
            {
                "source": "yardcare.md",
                "question": "Rosie, what plants are in the raised bed according to the yard care plan?",
                "expected_keywords": ["purple heart", "salvia"],
                "expected_answer": "Purple Heart and Salvia"
            },
        ]

        # Randomly select a test case
        test_case = random.choice(test_cases)

        print("\n" + "="*70)
        print("ROSIE RAG KNOWLEDGE BASE TEST")
        print("="*70)
        print(f"\nTesting retrieval from: {test_case['source']}")
        print(f"Question: {test_case['question']}")
        print(f"Expected answer: {test_case['expected_answer']}")
        print()

        # Add question to conversation history
        timestamp = datetime.datetime.now().strftime('%I:%M %p')
        history_entry = f"Human [{timestamp}]: {test_case['question']}\n"
        with open(self.history_file, 'a') as f:
            f.write(history_entry)

        # Get ROSIE's answer (text only, no TTS)
        self._ollama_response()
        answer_response = self.speak_file.read_text().strip()

        # Add ROSIE's answer to history
        timestamp = datetime.datetime.now().strftime('%I:%M %p')
        history_entry = f"Robot [{timestamp}]: {answer_response}\n"
        with open(self.history_file, 'a') as f:
            f.write(history_entry)

        print(f"ROSIE answer: {answer_response}\n")

        print("-"*70)
        print("\nEvaluating response accuracy...\n")

        # Check if response contains expected keywords
        # Keywords are treated as alternatives - finding ANY one of them means success
        answer_lower = answer_response.lower()
        keywords_found = []

        for keyword in test_case['expected_keywords']:
            if keyword.lower() in answer_lower:
                keywords_found.append(keyword)

        # Success if at least one keyword variant was found
        passed = len(keywords_found) > 0

        print(f"Expected (any of): {test_case['expected_keywords']}")
        print(f"Found: {keywords_found if keywords_found else 'None'}")

        print("\n" + "="*70)
        if passed:
            print("TEST RESULT: ✓ PASSED - RAG retrieval successful")
            result = True
        else:
            print("TEST RESULT: ✗ FAILED - Expected answer not found in response")
            print("(Check that RAG knowledge base is properly indexed)")
            result = False
        print("="*70 + "\n")

        return result

    def _run_test_calendar(self):
        """
        Run Google Calendar accuracy test.

        Dynamically reads calendar_events.md to find events for today and the upcoming week,
        then generates a question about a randomly selected event and verifies the answer.
        """
        import random
        import datetime
        import re

        def parse_calendar_events(calendar_path):
            """Parse calendar_events.md and return list of events with dates."""
            events = []
            current_date = None
            current_date_str = ""

            try:
                with open(calendar_path, 'r') as f:
                    content = f.read()

                # Pattern to match date headers: ## Day, Month DD, YYYY
                date_pattern = re.compile(r'^## (\w+), (\w+) (\d{2}), (\d{4})$', re.MULTILINE)
                # Pattern to match events: - **event name** ...
                event_pattern = re.compile(r'^- \*\*(.+?)\*\*', re.MULTILINE)
                # Pattern to match time: at HH:MM AM/PM
                time_pattern = re.compile(r'at (\d{1,2}:\d{2} [AP]M)')

                lines = content.split('\n')
                i = 0
                while i < len(lines):
                    line = lines[i].strip()

                    # Check for date header
                    date_match = date_pattern.match(line)
                    if date_match:
                        day_name, month_name, day, year = date_match.groups()
                        # Parse the date
                        try:
                            current_date = datetime.datetime.strptime(
                                f"{month_name} {day}, {year}", "%B %d, %Y"
                            ).date()
                            current_date_str = f"{day_name}, {month_name} {day}, {year}"
                        except ValueError:
                            current_date = None
                        i += 1
                        continue

                    # Check for event (skip metadata lines like "When:" and "Location:")
                    event_match = event_pattern.match(line)
                    if event_match and current_date:
                        event_name = event_match.group(1).strip()
                        # Skip metadata lines
                        if event_name.lower() in ['when:', 'location:']:
                            i += 1
                            continue
                        # Look for time in next few lines
                        event_time = None
                        for j in range(i, min(i + 3, len(lines))):
                            time_match = time_pattern.search(lines[j])
                            if time_match:
                                event_time = time_match.group(1)
                                break

                        events.append({
                            'name': event_name,
                            'date': current_date,
                            'date_str': current_date_str,
                            'time': event_time,
                            'day_name': current_date.strftime('%A')
                        })

                    i += 1

            except Exception as e:
                debug_print(f"[CALENDAR TEST] Error parsing calendar: {e}")

            return events

        def extract_keywords(event):
            """
            Extract searchable keywords from an event.

            Returns two lists:
            - name_keywords: Keywords from the event name (REQUIRED for test to pass)
            - temporal_keywords: Day/month/date keywords (supplementary, not sufficient alone)

            The test must find at least one name_keyword to pass. Finding only temporal
            keywords (like 'saturday') is not enough because ROSIE might say
            "nothing on Saturday" which would incorrectly pass.
            """
            # Event name keywords - at least one of these MUST be found
            skip_words = {'the', 'a', 'an', 'to', 'for', 'and', 'or', 'in', 'on', 'at', 'is',
                         'fw:', 'fw', 're:', 're', 'pm', 'am', 'reminder'}
            name_words = re.findall(r'\b[a-zA-Z]{3,}\b', event['name'].lower())
            name_keywords = [w for w in name_words if w not in skip_words][:4]

            # Temporal keywords - supplementary info
            temporal_keywords = []
            temporal_keywords.append(event['day_name'].lower())
            temporal_keywords.append(event['date'].strftime('%B').lower())
            temporal_keywords.append(str(event['date'].day))
            if event['time']:
                temporal_keywords.append(event['time'].lower().replace(' ', ''))

            return name_keywords, temporal_keywords

        # Get today's date and calculate week range
        today = datetime.date.today()
        week_end = today + datetime.timedelta(days=7)

        # Parse calendar events
        calendar_path = self.knowledge_base_dir / 'calendar_events.md'
        all_events = parse_calendar_events(calendar_path)

        if not all_events:
            print("\n" + "="*70)
            print("ROSIE CALENDAR ACCURACY TEST")
            print("="*70)
            print("\nERROR: No events found in calendar_events.md")
            print("Make sure calendar is synced before running this test.")
            print("="*70 + "\n")
            return False

        # Filter events for today and upcoming week
        week_events = [e for e in all_events if today <= e['date'] <= week_end]
        today_events = [e for e in all_events if e['date'] == today]

        # If no events this week, use all future events
        if not week_events:
            week_events = [e for e in all_events if e['date'] >= today]

        if not week_events:
            print("\n" + "="*70)
            print("ROSIE CALENDAR ACCURACY TEST")
            print("="*70)
            print("\nERROR: No upcoming events found in calendar")
            print("="*70 + "\n")
            return False

        # Randomly select an event
        selected_event = random.choice(week_events)

        # Generate questions - mix of day-based and event-name-based
        # The validation requires event NAME keywords to pass, so if we ask
        # "what is happening on Saturday?" and there's a graduation party,
        # ROSIE must mention "graduation" or "party" - not just "Saturday"

        event_name = selected_event['name']
        day_name = selected_event['day_name']

        # Extract searchable terms from the event name (2-4 significant words)
        skip_words = {'the', 'for', 'and', 'with', 'from', 'this', 'that', 'your'}
        name_words = [w for w in event_name.split() if len(w) > 2 and w.lower() not in skip_words][:4]
        short_name = ' '.join(name_words)

        # Mix of question types:
        # 1. Day-based questions - tests if ROSIE knows what's on a specific day
        # 2. Event-name questions - tests if ROSIE can find a specific event
        # Include "this week" or date context to help RAG find the right week
        date_str = selected_event['date'].strftime('%B %d')  # e.g., "December 13"

        question_templates = [
            # Day-based questions with context (ROSIE must respond with the actual event name)
            f"Rosie, what is happening this {day_name}?",
            f"Rosie, do I have anything scheduled for {day_name} this week?",
            f"Rosie, what is on the calendar for {day_name}, {date_str}?",
            # Event-name questions
            f"Rosie, when is {short_name} on the calendar?",
            f"Rosie, what day is {short_name} scheduled?",
        ]

        # Pick a random question
        question = random.choice(question_templates)

        # Extract expected keywords (name keywords are REQUIRED, temporal are supplementary)
        name_keywords, temporal_keywords = extract_keywords(selected_event)

        # Build expected answer description
        time_str = f" at {selected_event['time']}" if selected_event['time'] else ""
        expected_answer = f"{selected_event['name']}{time_str} on {selected_event['date_str']}"

        print("\n" + "="*70)
        print("ROSIE CALENDAR ACCURACY TEST")
        print("="*70)
        print(f"\nToday's date: {today.strftime('%A, %B %d, %Y')}")
        print(f"Selected event: {selected_event['name']}")
        print(f"Event date: {selected_event['date_str']}")
        if selected_event['time']:
            print(f"Event time: {selected_event['time']}")
        print(f"\nQuestion: {question}")
        print(f"Expected answer: {expected_answer}")
        print()

        # Add question to conversation history
        timestamp = datetime.datetime.now().strftime('%I:%M %p')
        history_entry = f"Human [{timestamp}]: {question}\n"
        with open(self.history_file, 'a') as f:
            f.write(history_entry)

        # Get ROSIE's answer (text only, no TTS)
        self._ollama_response()
        answer_response = self.speak_file.read_text().strip()

        # Add ROSIE's answer to history
        timestamp = datetime.datetime.now().strftime('%I:%M %p')
        history_entry = f"Robot [{timestamp}]: {answer_response}\n"
        with open(self.history_file, 'a') as f:
            f.write(history_entry)

        print(f"ROSIE answer: {answer_response}\n")

        print("-"*70)
        print("\nEvaluating response accuracy...\n")

        # Check if response contains expected keywords
        # CRITICAL: At least one NAME keyword must be found
        # Temporal keywords alone are NOT sufficient (e.g., "nothing on Saturday" would incorrectly pass)
        answer_lower = answer_response.lower()

        name_keywords_found = []
        for keyword in name_keywords:
            if keyword.lower() in answer_lower:
                name_keywords_found.append(keyword)

        temporal_keywords_found = []
        for keyword in temporal_keywords:
            if keyword.lower() in answer_lower:
                temporal_keywords_found.append(keyword)

        # Success ONLY if at least one NAME keyword is found
        # This prevents false positives like "nothing scheduled for Saturday"
        passed = len(name_keywords_found) > 0

        print(f"Required (event name): {name_keywords}")
        print(f"Supplementary (date/time): {temporal_keywords}")
        print(f"Name keywords found: {name_keywords_found if name_keywords_found else 'None'}")
        print(f"Temporal keywords found: {temporal_keywords_found if temporal_keywords_found else 'None'}")

        print("\n" + "="*70)
        if passed:
            print("TEST RESULT: ✓ PASSED - Calendar retrieval successful")
            result = True
        else:
            print("TEST RESULT: ✗ FAILED - Event name not found in response")
            if temporal_keywords_found and not name_keywords_found:
                print("(ROSIE mentioned the date/time but not the actual event)")
            print("(Check that calendar_events.md is properly synced and indexed)")
            result = False
        print("="*70 + "\n")

        return result

    def _run_test_conversation(self):
        """
        Run conversation quality test.

        Tests ROSIE's intent classification and response appropriateness for:
        - Greetings (should get casual responses, no schedule talk)
        - Farewells (should get brief goodbyes)
        - Acknowledgments (should get very brief responses)
        - Questions (should get factual answers)
        - General conversation (should get natural responses)
        """
        import datetime

        print("\n" + "="*70)
        print("ROSIE CONVERSATION QUALITY TEST")
        print("="*70)
        print("\nThis test verifies intent classification and response quality.")
        print("Each test checks that ROSIE responds appropriately to different message types.\n")

        # Test cases with expected intent and validation criteria
        test_cases = [
            {
                "category": "GREETING",
                "input": "Hey, what is going on?",
                "expected_intent": "greeting",
                "should_not_contain": ["schedule", "meeting", "appointment", "calendar", "event"],
                "should_be_brief": True,
                "max_words": 25,
                "description": "Casual greeting should get casual response, no schedule talk"
            },
            {
                "category": "GREETING",
                "input": "How's it going?",
                "expected_intent": "greeting",
                "should_not_contain": ["schedule", "meeting", "appointment", "calendar"],
                "should_be_brief": True,
                "max_words": 20,
                "description": "Simple greeting should get brief, friendly response"
            },
            {
                "category": "ACKNOWLEDGMENT",
                "input": "No, that's okay.",
                "expected_intent": "acknowledgment",
                "should_not_contain": ["how about you", "how are you", "schedule", "meeting"],
                "should_be_brief": True,
                "max_words": 10,
                "description": "Acknowledgment should get very brief response, not a question"
            },
            {
                "category": "ACKNOWLEDGMENT",
                "input": "Thanks",
                "expected_intent": "acknowledgment",
                "should_contain_any": ["welcome", "no problem", "sure", "anytime", "glad"],
                "should_be_brief": True,
                "max_words": 10,
                "description": "Thanks should get brief acknowledgment"
            },
            {
                "category": "FAREWELL",
                "input": "Bye, talk later!",
                "expected_intent": "farewell",
                "should_contain_any": ["bye", "later", "see you", "take care", "catch", "soon"],
                "should_be_brief": True,
                "max_words": 15,
                "description": "Farewell should get brief goodbye"
            },
            {
                "category": "QUESTION",
                "input": "What time is it?",
                "expected_intent": "question",
                "should_contain_any": [":", "am", "pm", "o'clock", "morning", "afternoon", "evening"],
                "should_be_brief": True,
                "max_words": 20,
                "description": "Time question should get time-related answer"
            },
            {
                "category": "CONVERSATION",
                "input": "I've been thinking about how technology is changing the way we communicate. "
                         "People used to write letters and wait weeks for a response, but now we can "
                         "video chat with anyone in the world instantly. What do you think about that?",
                "expected_intent": "question",  # Ends with "?" so classified as question - that's fine
                "min_words": 15,
                "max_words": 100,
                "should_not_contain": ["schedule", "calendar", "appointment", "meeting"],
                "should_engage": True,
                "description": "Topic discussion should get thoughtful, engaged response"
            },
            {
                "category": "CONVERSATION",
                "input": "I had a really tough day at work today. My project deadline got moved up "
                         "and I'm feeling pretty stressed about it.",
                "expected_intent": "conversation",  # Statement, not a question
                "min_words": 10,
                "max_words": 60,
                "should_not_contain": ["schedule", "calendar", "I don't know", "try to", "you should",
                                       "you could", "have you tried", "breathing", "exercise", "meditation",
                                       "would you like me to"],
                "should_contain_any": ["sorry", "hear", "sounds", "tough", "rough", "hard", "stressful",
                                       "understand", "that's", "must be"],
                "should_engage": True,
                "description": "Emotional sharing should get empathetic response, NOT advice"
            },
        ]

        results = []
        passed_count = 0
        failed_count = 0

        for i, test in enumerate(test_cases, 1):
            print(f"\n{'─'*70}")
            print(f"TEST {i}/{len(test_cases)}: {test['category']}")
            print(f"{'─'*70}")
            print(f"Input: \"{test['input']}\"")
            print(f"Expected intent: {test['expected_intent']}")
            print(f"Criteria: {test['description']}")
            print()

            # Clear conversation history for clean test
            self.history_file.write_text("")

            # Classify intent
            actual_intent = self._classify_intent(test['input'])
            intent_correct = actual_intent == test['expected_intent']

            print(f"Actual intent: {actual_intent} {'✓' if intent_correct else '✗'}")

            # Add to conversation history and get response
            timestamp = datetime.datetime.now().strftime('%I:%M %p')
            history_entry = f"Human [{timestamp}]: {test['input']}\n"
            with open(self.history_file, 'a') as f:
                f.write(history_entry)

            # Get ROSIE's response
            self._ollama_response()
            response = self.speak_file.read_text().strip()

            print(f"Response: \"{response}\"")

            # Validate response
            response_lower = response.lower()
            word_count = len(response.split())
            validation_passed = True
            validation_notes = []

            # Check intent classification
            if not intent_correct:
                validation_passed = False
                validation_notes.append(f"Wrong intent: expected '{test['expected_intent']}', got '{actual_intent}'")

            # Check should_not_contain
            if 'should_not_contain' in test:
                for word in test['should_not_contain']:
                    if word.lower() in response_lower:
                        validation_passed = False
                        validation_notes.append(f"Response contains forbidden word: '{word}'")

            # Check should_contain_any
            if 'should_contain_any' in test:
                found_any = any(word.lower() in response_lower for word in test['should_contain_any'])
                if not found_any:
                    validation_passed = False
                    validation_notes.append(f"Response missing expected words: {test['should_contain_any']}")

            # Check brevity (max words)
            if test.get('should_be_brief') and 'max_words' in test:
                if word_count > test['max_words']:
                    validation_passed = False
                    validation_notes.append(f"Response too long: {word_count} words (max {test['max_words']})")

            # Check minimum length for conversation tests
            if 'min_words' in test:
                if word_count < test['min_words']:
                    validation_passed = False
                    validation_notes.append(f"Response too short: {word_count} words (min {test['min_words']})")

            # Check max_words without should_be_brief (for conversation tests)
            if 'max_words' in test and not test.get('should_be_brief'):
                if word_count > test['max_words']:
                    validation_passed = False
                    validation_notes.append(f"Response too long: {word_count} words (max {test['max_words']})")

            # Record result
            if validation_passed:
                print(f"\nResult: ✓ PASSED")
                passed_count += 1
            else:
                print(f"\nResult: ✗ FAILED")
                for note in validation_notes:
                    print(f"  - {note}")
                failed_count += 1

            results.append({
                "test": test['category'],
                "input": test['input'],
                "passed": validation_passed,
                "notes": validation_notes
            })

        # Summary
        print("\n" + "="*70)
        print("CONVERSATION TEST SUMMARY")
        print("="*70)
        print(f"\nTotal tests: {len(test_cases)}")
        print(f"Passed: {passed_count}")
        print(f"Failed: {failed_count}")

        if failed_count == 0:
            print("\n✓ ALL TESTS PASSED - Conversation quality is good!")
            overall_result = True
        else:
            print(f"\n✗ {failed_count} TEST(S) FAILED - Review the issues above")
            overall_result = False

        print("="*70 + "\n")

        return overall_result

    def _process_test_input(self, text):
        """
        Process text input in test mode - FULL AUDIO PIPELINE TEST

        Tests the complete audio flow:
        1. Text → Piper TTS → Audio generation
        2. Audio → Speakers (aplay playback)
        3. Speakers → Microphone → ROSIE capture
        4. Audio → VAD → Speech detection
        5. Speech → Whisper → Transcription
        6. Transcription validation and accuracy check
        7. If wake word detected → Continue to LLM response

        Args:
            text: The text to synthesize and speak through the audio pipeline
        """
        print("\n" + "="*70)
        debug_print(f"[TEST] Full Audio Pipeline Test")
        print("="*70)
        debug_print(f"[TEST] Input text: \"{text}\"")
        debug_print(f"[TEST] Starting audio pipeline: Text → Piper → Speakers → Microphone → VAD → Whisper")
        print("="*70)

        # Reset transcription capture
        with self.test_transcription_lock:
            self.test_last_transcription = None
            self.test_transcription_event.clear()

        # Step 1: Generate audio with Piper
        print(f"\n[TEST STEP 1] Generating audio with Piper...")
        try:
            # Escape text for shell safety (same as production code)
            text_escaped = text.replace('"', '\\"').replace('$', '\\$').replace('`', '\\`')

            # Use same Piper command as production (LOCAL mode)
            piper_cmd = f'echo "{text_escaped}" | piper --model {self.piper_model_path} --output-raw | aplay -r 22050 -f S16_LE -t raw -'
            debug_print(f"[TEST] Piper command: {piper_cmd[:80]}...")

            # Step 2: Play through speakers (this will take ~2-5 seconds depending on text length)
            print(f"[TEST STEP 2] Playing audio through speakers...")
            result = subprocess.run(piper_cmd, shell=True, capture_output=True, text=True)

            if result.returncode != 0:
                print(f"[TEST ERROR] Piper/aplay failed: {result.stderr}")
                return

            debug_print(f"[TEST] Audio playback complete")

        except Exception as e:
            print(f"[TEST ERROR] Piper audio generation failed: {e}")
            import traceback
            traceback.print_exc()
            return

        # Step 3: Wait for ROSIE to capture, process, and transcribe
        print(f"\n[TEST STEP 3] Waiting for ROSIE to capture audio via microphone...")
        debug_print(f"[TEST] (Microphone → VAD → Audio buffering → Silence detection → Whisper)")

        # Give a short delay for final audio to be captured, then force transcription
        time.sleep(2.0)  # Allow remaining audio to be captured
        self.test_force_transcribe.set()  # Signal whisper worker to transcribe now

        # Wait up to 20 seconds for transcription
        transcription_received = self.test_transcription_event.wait(timeout=20.0)

        if not transcription_received:
            print(f"\n[TEST FAILURE] No transcription received within 20 seconds")
            debug_print(f"[TEST] This indicates a problem in the audio pipeline:")
            debug_print(f"[TEST]   - VAD may not be detecting speech")
            debug_print(f"[TEST]   - Audio buffering may not be working")
            debug_print(f"[TEST]   - Silence detection may not be triggering Whisper")
            debug_print(f"[TEST]   - Whisper may not be transcribing")
            return

        # Step 4: Retrieve and validate transcription
        with self.test_transcription_lock:
            transcribed_text = self.test_last_transcription

        print(f"\n[TEST STEP 4] Transcription received!")
        debug_print(f"[TEST] Original:    \"{text}\"")
        debug_print(f"[TEST] Transcribed: \"{transcribed_text}\"")

        # Step 5: Calculate accuracy
        accuracy, word_accuracy = self._calculate_transcription_accuracy(text, transcribed_text)

        print(f"\n[TEST RESULTS]")
        print(f"  Character accuracy: {accuracy:.1f}%")
        print(f"  Word accuracy:      {word_accuracy:.1f}%")

        if accuracy >= 90:
            print(f"  Status: ✓ EXCELLENT - Audio pipeline working correctly")
        elif accuracy >= 70:
            print(f"  Status: ✓ GOOD - Minor transcription differences")
        elif accuracy >= 50:
            print(f"  Status: ⚠ FAIR - Significant transcription errors")
        else:
            print(f"  Status: ✗ POOR - Audio pipeline may have issues")

        print("="*70)

        # Check if wake word was detected (informational only)
        print(f"\n[TEST] Checking for wake word in transcription...")
        wake_word_pattern = r'\b(rosie|rose|rosy|rosee)\b'
        wake_word_match = re.search(wake_word_pattern, transcribed_text, re.IGNORECASE)

        if wake_word_match:
            debug_print(f"[TEST] ✓ Wake word 'Rosie' detected in transcription")
        else:
            debug_print(f"[TEST] ✗ Wake word not detected")
            debug_print(f"[TEST]   (Include 'Rosie' in test text to verify wake word detection)")

        print(f"\n[TEST] Audio pipeline test complete!")
        print("="*70 + "\n")

    def _wake_word_detector(self):
        """
        Wake word detection worker thread

        Monitors wake_word_detected flag and triggers response.
        Flag is set by Whisper when wake word detected.
        Active only during LISTENING state.
        """
        self._log("Wake word detector started")

        while not self.shutdown_event.is_set():
            current_state = self._get_state()

            # Only active in LISTENING state (and not in test mode)
            if current_state == ConversationState.LISTENING and not self.test_mode:
                try:
                    # Check if wake word flag was set by Whisper
                    with self.wake_word_lock:
                        if self.wake_word_detected:
                            # Reset flag
                            self.wake_word_detected = False

                            # Always print wake word detection to console
                            print(f"\n[WAKE WORD] Detected! Processing...")
                            self._log(f"Wake word flag detected!")

                            # Read conversation history for command detection
                            content = self.history_file.read_text()

                            # Check for demo mode FIRST (before memory reset)
                            demo_pattern = r'\bdemo\b'
                            demo_match = re.search(demo_pattern, content, re.IGNORECASE)

                            if demo_match:
                                print(f"\n[DEMO MODE] Detected! Starting AI conversation mode...")
                                # Clear conversation history
                                self.history_file.write_text('')
                                # Enable demo mode (auto-respond without wake word)
                                with self.demo_mode_lock:
                                    self.demo_mode = True
                                print(f"[DEMO MODE] Auto-response enabled (no wake word required)")
                                # Set demo response
                                ai_intro = "Ok, I'm ready! Five, four, three, two, one. Hello my name is Rosie and I am an AI. I would like to have a conversation with you!"
                                self.speak_file.write_text(ai_intro)
                                print(f"[DEMO MODE] Robot will say: {ai_intro}")
                                # Transition to SPEAKING state
                                self._set_state(ConversationState.SPEAKING)
                                # Trigger speech output
                                threading.Thread(target=self._piper_speak, daemon=True).start()
                                continue

                            # Check for memory reset command in recent history
                            forget_pattern = r'\b(forget everything|clear memory|reset memory)\b'
                            forget_match = re.search(forget_pattern, content, re.IGNORECASE)

                            if forget_match:
                                print(f"\n[MEMORY] Reset command detected! Clearing conversation history...")
                                self.history_file.write_text('')
                                print(f"[MEMORY] Conversation history cleared. Starting fresh.")
                                continue

                            # Transition to RESPONDING state
                            self._set_state(ConversationState.RESPONDING)

                            # Trigger Ollama response (in new thread to avoid blocking)
                            threading.Thread(target=self._ollama_response, daemon=True).start()

                except Exception as e:
                    self._log(f"Wake word detection error: {e}")

            # Check every 100ms
            time.sleep(0.1)

        self._log("Wake word detector stopped")

    def _analyze_conversation_depth(self, conversation_content, last_human_statement):
        """
        Analyze conversation depth to determine appropriate response length

        Args:
            conversation_content: Full conversation history text
            last_human_statement: Most recent human statement

        Returns:
            str: 'shallow', 'medium', or 'deep'
        """
        # Count total exchanges (Human + Robot pairs)
        human_lines = [line for line in conversation_content.split('\n') if line.startswith('Human ')]
        robot_lines = [line for line in conversation_content.split('\n') if line.startswith('Robot ')]
        exchange_count = min(len(human_lines), len(robot_lines))

        # Extract the actual text from last human statement (remove timestamp prefix)
        last_text = re.sub(r'^Human\s+\[.*?\]:\s*', '', last_human_statement).strip().lower()

        # SHALLOW indicators - First priority

        # Very first exchange
        if exchange_count < 2:
            return 'shallow'

        # Simple greetings
        greeting_patterns = [r'\b(hello|hi|hey|good morning|good afternoon|good evening)\b']
        if any(re.search(pattern, last_text) for pattern in greeting_patterns):
            return 'shallow'

        # Single fact questions (what time, what's the weather, etc.)
        simple_fact_patterns = [
            r'\b(what time|what\'s the time|what date)\b',
            r'\b(what\'s the weather|weather)\b',
            r'^\w+\?$'  # Single word questions
        ]
        if any(re.search(pattern, last_text) for pattern in simple_fact_patterns):
            return 'shallow'

        # Topic change detection - compare last 2-3 human statements
        if len(human_lines) >= 3:
            # Extract last 3 human statements
            recent_statements = human_lines[-3:]
            recent_texts = [re.sub(r'^Human\s+\[.*?\]:\s*', '', s).strip().lower() for s in recent_statements]

            # Simple keyword overlap check between last statement and previous ones
            last_words = set(re.findall(r'\b\w+\b', recent_texts[-1]))
            previous_words = set()
            for text in recent_texts[:-1]:
                previous_words.update(re.findall(r'\b\w+\b', text))

            # Remove common stop words for better comparison
            stop_words = {'the', 'a', 'an', 'is', 'are', 'was', 'were', 'what', 'how', 'when', 'where', 'who', 'why', 'do', 'does', 'did', 'can', 'could', 'would', 'should', 'i', 'you', 'me', 'my', 'your', 'it', 'that', 'this'}
            last_words = last_words - stop_words
            previous_words = previous_words - stop_words

            # If less than 15% word overlap, likely a topic change
            if len(last_words) > 0:
                overlap = len(last_words & previous_words) / len(last_words)
                if overlap < 0.15:
                    return 'shallow'

        # DEEP indicators - Second priority

        # 5+ consecutive exchanges
        if exchange_count >= 5:
            # Additional deep indicators
            deep_patterns = [
                r'\b(also|furthermore|additionally|moreover|besides)\b',
                r'\b(but what about|what if|how about)\b',
                r'\b(what do you think|why does|how does|why do|what makes)\b',
                r'\b(explain|tell me more|elaborate|describe)\b',
                # Pronouns referring to previous context
                r'\b(it|that|they|them|those|these)\b.*\?',
                # Why/How questions
                r'\b(why|how)\b.*\?'
            ]

            if any(re.search(pattern, last_text) for pattern in deep_patterns):
                return 'deep'

        # Follow-up questions with pronouns (even with fewer exchanges)
        pronoun_followup_patterns = [
            r'\b(it|that|they|them|those|these)\b.*\?',
            r'^(and |but |so )',  # Sentence continuation
            r'\b(what about it|why is that|how does that)\b'
        ]
        if any(re.search(pattern, last_text) for pattern in pronoun_followup_patterns):
            return 'deep'

        # Why/How questions (complex reasoning)
        if re.search(r'\b(why|how)\b.*\?', last_text):
            return 'deep'

        # Continuity words
        continuity_patterns = [
            r'\b(also|furthermore|additionally|moreover|besides)\b',
            r'\b(but what about|what if|how about)\b',
            r'\b(tell me more|elaborate|explain further)\b'
        ]
        if any(re.search(pattern, last_text) for pattern in continuity_patterns):
            return 'deep'

        # Requests for opinions/reasoning
        opinion_patterns = [
            r'\b(what do you think|do you think|your opinion|what would you)\b',
            r'\b(why does that|what makes that|how come)\b'
        ]
        if any(re.search(pattern, last_text) for pattern in opinion_patterns):
            return 'deep'

        # MEDIUM is the default for everything in between
        return 'medium'

    def _keyboard_listener(self):
        """
        Keyboard listener thread for spacebar wake trigger

        Monitors keyboard for spacebar press as alternative to voice wake word.
        Only active during LISTENING state in normal (non-text) mode.
        """
        import sys
        import select

        # Try to set up terminal for raw input (Unix-specific)
        try:
            import termios
            import tty

            # Save original terminal settings
            old_settings = termios.tcgetattr(sys.stdin)
            self._log("Keyboard listener started (spacebar = wake word)")

            try:
                # Set terminal to raw mode (character-by-character input)
                tty.setcbreak(sys.stdin.fileno())

                while not self.shutdown_event.is_set():
                    current_state = self._get_state()

                    # Only active in LISTENING state
                    if current_state == ConversationState.LISTENING:
                        # Check if input is available (non-blocking)
                        if select.select([sys.stdin], [], [], 0.1)[0]:
                            char = sys.stdin.read(1)

                            # Spacebar pressed
                            if char == ' ':
                                with self.wake_word_lock:
                                    self.wake_word_detected = True
                                debug_print("[KEYBOARD] Spacebar pressed - wake trigger!")
                    else:
                        # Not in LISTENING state, just sleep
                        time.sleep(0.1)

            finally:
                # Restore terminal settings
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

        except Exception as e:
            # Terminal raw mode not available (e.g., not a TTY)
            debug_print(f"[KEYBOARD] Keyboard listener not available: {e}")
            return

    def _classify_intent(self, text):
        """
        Classify user message intent for appropriate response routing

        Args:
            text: The user's message text (without timestamp prefix)

        Returns:
            str: 'greeting', 'farewell', 'acknowledgment', 'question', 'task', or 'conversation'
        """
        text_lower = text.lower().strip()

        # GREETING patterns - casual hellos and "how are you" type messages
        greeting_patterns = [
            r'^(hey|hi|hello|howdy|yo)[\s,!?.]*$',  # Just "hey", "hi", etc.
            r'^(hey|hi|hello|howdy)\s+(there|rosie|rose)[\s,!?.]*$',  # "Hey there", "Hi Rosie"
            r'^good\s+(morning|afternoon|evening|day)[\s,!?.]*$',  # Time-based greetings
            r'^what(\'?s|\s+is)\s+(up|going\s+on|happening)[\s,!?.]*$',  # "What's up", "What is going on"
            r'^how(\'?s|\s+is)\s+it\s+going[\s,!?.]*$',  # "How's it going", "How is it going"
            r'^sup[\s,!?.]*$',  # "Sup"
            r'^how\s+(are\s+you|you\s+doing|have\s+you\s+been)[\s,!?.]*$',  # "How are you" variants
            r'^(hey|hi|hello)[\s,]+.{0,10}(what|how)',  # "Hey, what's up", "Hey, what is going on"
        ]
        if any(re.search(pattern, text_lower) for pattern in greeting_patterns):
            return 'greeting'

        # FAREWELL patterns
        farewell_patterns = [
            r'\b(bye|goodbye|good\s*bye|see\s+you|talk\s+(later|soon)|gotta\s+go|have\s+to\s+go)\b',
            r'^(later|peace|take\s+care)[\s,!?.]*$',
        ]
        if any(re.search(pattern, text_lower) for pattern in farewell_patterns):
            return 'farewell'

        # ACKNOWLEDGMENT patterns - responses to ROSIE's previous message
        acknowledgment_patterns = [
            r'^(ok(ay)?|alright|sure|got\s+it|understood|i\s+see|makes\s+sense)[\s,!?.]*$',
            r'^(thanks|thank\s+you|thx)[\s,!?.]*$',
            r'^(no|nope|nah)[\s,]*(thanks|thank\s+you|that\'?s?\s+(ok(ay)?|fine|alright))[\s,!?.]*$',
            r'^(that\'?s?\s+)?(ok(ay)?|fine|alright|good|great)[\s,!?.]*$',
            r'^(never\s*mind|forget\s+it|don\'?t\s+worry)[\s,!?.]*$',
            r'^(yes|yeah|yep|yup|no|nope|nah)[\s,!?.]*$',  # Simple yes/no
            r'^no[\s,]+.{0,15}(ok(ay)?|fine|alright)',  # "No, that's okay"
        ]
        if any(re.search(pattern, text_lower) for pattern in acknowledgment_patterns):
            return 'acknowledgment'

        # TASK patterns - action requests
        task_patterns = [
            r'\b(schedule|remind|set\s+up|create|add|book|make)\b.*\b(appointment|meeting|event|reminder|alarm)\b',
            r'\b(turn\s+(on|off)|switch|open|close|start|stop)\b',
            r'\b(remind\s+me|set\s+(a\s+)?(timer|alarm|reminder))\b',
        ]
        if any(re.search(pattern, text_lower) for pattern in task_patterns):
            return 'task'

        # QUESTION patterns - factual queries
        question_patterns = [
            r'\b(what|when|where|who|which|how\s+many|how\s+much|how\s+long|how\s+far)\b.*\?',
            r'^(is|are|was|were|do|does|did|can|could|will|would|should|have|has|had)\b.*\?',
            r'\b(tell\s+me\s+about|what\'?s?\s+the)\b',
        ]
        if any(re.search(pattern, text_lower) for pattern in question_patterns):
            return 'question'

        # Check for question mark at end (catch-all for questions)
        if text_lower.rstrip().endswith('?'):
            return 'question'

        # Default to general conversation
        return 'conversation'


    # =====================================================================
    # STATE 2: RESPONDING - Ollama Immediate Response
    # =====================================================================

    def _ollama_response(self):
        """
        Ollama response generation using full conversation context

        Automatically summarizes context if it exceeds token limit.
        """
        self._log("Ollama processing started")

        # Update last activity timestamp (for private mode timeout)
        with self.private_mode_lock:
            self.last_activity_timestamp = time.time()

        try:
            # Read conversation history
            conversation_content = self.history_file.read_text()

            # Extract last human statement first (needed for intent classification)
            human_lines = [line for line in conversation_content.split('\n') if line.startswith('Human ')]
            last_human_statement = human_lines[-1] if human_lines else ""
            question = re.sub(r'^Human\s+\[.*?\]:\s*', '', last_human_statement) if last_human_statement else ""

            # Classify intent to determine response strategy
            intent = self._classify_intent(question)
            debug_print(f"[INTENT] Classified as: {intent}")

            # Query RAG knowledge base ONLY for questions and tasks (not casual conversation)
            rag_context = ""
            if self.rag_system and intent in ['question', 'task']:
                if question:
                    # Enhance calendar-related queries with current date context
                    # This helps RAG retrieve the correct week's events instead of any matching day
                    import datetime
                    enhanced_question = question
                    date_filter = None  # Will be set if we can determine the target date
                    day_names = ['monday', 'tuesday', 'wednesday', 'thursday', 'friday', 'saturday', 'sunday']
                    question_lower = question.lower()
                    today = datetime.date.today()

                    # Check for "today" or "tonight"
                    if 'today' in question_lower or 'tonight' in question_lower:
                        target_date = today
                        date_filter = target_date.strftime('%B %d, %Y')  # e.g., "December 09, 2025"
                        enhanced_question = f"{question} {date_filter}"
                        debug_print(f"[RAG] Enhanced query with today's date: {enhanced_question}")
                        debug_print(f"[RAG] Date filter set: {date_filter}")

                    # Check for "tomorrow"
                    elif 'tomorrow' in question_lower:
                        target_date = today + datetime.timedelta(days=1)
                        date_filter = target_date.strftime('%B %d, %Y')  # e.g., "December 10, 2025"
                        enhanced_question = f"{question} {date_filter}"
                        debug_print(f"[RAG] Enhanced query with tomorrow's date: {enhanced_question}")
                        debug_print(f"[RAG] Date filter set: {date_filter}")

                    # Check for "yesterday"
                    elif 'yesterday' in question_lower:
                        target_date = today - datetime.timedelta(days=1)
                        date_filter = target_date.strftime('%B %d, %Y')
                        enhanced_question = f"{question} {date_filter}"
                        debug_print(f"[RAG] Enhanced query with yesterday's date: {enhanced_question}")
                        debug_print(f"[RAG] Date filter set: {date_filter}")

                    # Check if question references a day of the week
                    else:
                        for day_name in day_names:
                            if day_name in question_lower:
                                # Calculate the actual date for "this [day]"
                                current_weekday = today.weekday()  # Monday=0, Sunday=6
                                target_weekday = day_names.index(day_name)

                                # Calculate days until target day (this week or next)
                                days_ahead = target_weekday - current_weekday
                                if days_ahead < 0:  # Target day already passed this week
                                    days_ahead += 7
                                target_date = today + datetime.timedelta(days=days_ahead)

                                # Set the date filter for RAG post-filtering
                                date_filter = target_date.strftime('%B %d, %Y')  # e.g., "December 09, 2025"
                                # Also add to query to help semantic search
                                enhanced_question = f"{question} {date_filter}"
                                debug_print(f"[RAG] Enhanced query with date: {enhanced_question}")
                                debug_print(f"[RAG] Date filter set: {date_filter}")
                                break

                    # Use more chunks for calendar/schedule queries to increase chance of finding right date
                    is_calendar_query = any(word in question_lower for word in ['calendar', 'scheduled', 'happening', 'appointment', 'event', 'monday', 'tuesday', 'wednesday', 'thursday', 'friday', 'saturday', 'sunday', 'today', 'tomorrow', 'week'])
                    top_k = 25 if is_calendar_query else 5

                    rag_context = self.rag_system.query(enhanced_question, top_k=top_k, date_filter=date_filter)

                    # Debug logging to show what RAG retrieved
                    if rag_context.strip():
                        debug_print(f"[RAG] Retrieved context ({len(rag_context)} chars)")
                        # Show first 200 chars of context for debugging
                        preview = rag_context[:200].replace('\n', ' ')
                        debug_print(f"[RAG] Preview: {preview}...")
                    else:
                        debug_print(f"[RAG] No relevant context found for: {question}")

            # Check if context exceeds limit - if so, summarize first
            # Note: important_info and RAG context are NOT included in token count for summarization
            # as they should always be preserved
            token_count = self._estimate_token_count(conversation_content)

            if token_count > self.context_limit:
                debug_print(f"[OLLAMA] Context exceeds limit! Summarizing...")
                self._speak_immediately("Let me think")

                # Create summary
                summary = self._summarize_conversation(conversation_content)
                if summary:
                    # Replace history with condensed script (no labels, just the script)
                    self.history_file.write_text(summary)
                    debug_print(f"[OLLAMA] History replaced with condensed script ({len(summary)} chars)")
                    conversation_content = self.history_file.read_text()
                else:
                    debug_print(f"[OLLAMA] Warning: Summarization failed, using full context")

            # Analyze conversation depth for dynamic response length
            # (last_human_statement already extracted above)
            conversation_depth = self._analyze_conversation_depth(conversation_content, last_human_statement)

            # Set dynamic token limits based on depth
            # Increased limits to allow proper knowledge base processing
            depth_token_limits = {
                'shallow': 150,   # 1-2 sentences (increased from 100 for KB extraction)
                'medium': 250,    # 2-3 sentences (increased from 200)
                'deep': 400       # 3-5 sentences with reasoning
            }
            max_tokens = depth_token_limits.get(conversation_depth, 250)

            # Display depth transitions or current depth
            if self.last_conversation_depth is None:
                # First conversation, just show depth
                debug_print(f"[{conversation_depth.upper()}] max_tokens: {max_tokens}")
            elif self.last_conversation_depth != conversation_depth:
                # Depth changed - show transition
                debug_print(f"[{self.last_conversation_depth.upper()} → {conversation_depth.upper()}] max_tokens: {max_tokens}")
            else:
                # Same depth - just show current
                debug_print(f"[{conversation_depth.upper()}] max_tokens: {max_tokens}")

            # Update last depth for next comparison
            self.last_conversation_depth = conversation_depth

            # Check for calendar event creation request (task intent handles this)
            if intent == 'task':
                calendar_create_pattern = r'\b(schedule|add|create|set up|book|make)\b.*\b(appointment|meeting|event|reminder)\b'
                is_calendar_create = bool(re.search(calendar_create_pattern, last_human_statement, re.IGNORECASE))
                if is_calendar_create:
                    self._handle_calendar_creation(last_human_statement, conversation_content)
                    return

            # Show what the user just said (unless in test modes)
            if not self.test_questions_mode and not self.test_knowledge_mode and not self.test_calendar_mode and not self.test_conversation_mode:
                print(f"\n→ You: {last_human_statement}")

            # Extract the actual text (without timestamp prefix)
            question_text = question  # Already extracted above

            # Get current date/time context (only needed for questions)
            import datetime
            now = datetime.datetime.now()

            # =====================================================================
            # INTENT-BASED PROMPT ROUTING
            # =====================================================================

            if intent == 'greeting':
                # GREETING: Simple, friendly response with few-shot examples
                temperature = 0.7
                max_tokens = 50  # Keep greetings short
                mode = "GREETING"

                prompt = (
                    "You are ROSIE, a friendly companion. Respond naturally to greetings.\n\n"
                    "Examples:\n"
                    "User: Hey, what's going on?\n"
                    "ROSIE: Not much! Just here if you need me. What's up?\n\n"
                    "User: How's it going?\n"
                    "ROSIE: Pretty good! How about you?\n\n"
                    "User: Good morning!\n"
                    "ROSIE: Good morning! How are you doing today?\n\n"
                    "User: Hi Rosie\n"
                    "ROSIE: Hey there! What's on your mind?\n\n"
                    f"User: {question_text}\n"
                    "ROSIE:"
                )

            elif intent == 'farewell':
                # FAREWELL: Brief, warm goodbye
                temperature = 0.7
                max_tokens = 30
                mode = "FAREWELL"

                prompt = (
                    "You are ROSIE. Say a brief, warm goodbye.\n\n"
                    "Examples:\n"
                    "User: Bye!\n"
                    "ROSIE: See you later!\n\n"
                    "User: Gotta go, talk later\n"
                    "ROSIE: Sounds good, catch you later!\n\n"
                    "User: Goodbye Rosie\n"
                    "ROSIE: Bye! Take care!\n\n"
                    f"User: {question_text}\n"
                    "ROSIE:"
                )

            elif intent == 'acknowledgment':
                # ACKNOWLEDGMENT: Very brief response to "okay", "thanks", etc.
                temperature = 0.5
                max_tokens = 25
                mode = "ACKNOWLEDGMENT"

                prompt = (
                    "You are ROSIE. The user just acknowledged or responded briefly. "
                    "Give a short, natural response (5 words or less).\n\n"
                    "Examples:\n"
                    "User: Okay\n"
                    "ROSIE: Sounds good!\n\n"
                    "User: Thanks\n"
                    "ROSIE: You're welcome!\n\n"
                    "User: No, that's okay\n"
                    "ROSIE: No problem!\n\n"
                    "User: Got it\n"
                    "ROSIE: Great!\n\n"
                    "User: Never mind\n"
                    "ROSIE: No worries!\n\n"
                    f"User: {question_text}\n"
                    "ROSIE:"
                )

            elif intent == 'question' or intent == 'task':
                # QUESTION/TASK: Full factual mode with RAG context
                temperature = 0.1
                mode = "FACTUAL"

                current_date_context = (
                    f"Current date/time: {now.strftime('%A, %B %d, %Y at %I:%M %p')}\n\n"
                )

                prompt = f"You are ROSIE, a helpful voice assistant.\n\n{current_date_context}"

                # Add RAG context if available
                if rag_context.strip():
                    prompt += (
                        "KNOWLEDGE BASE:\n"
                        f"{rag_context}\n\n"
                    )

                prompt += (
                    f"Recent conversation:\n{conversation_content}\n\n"
                    "Instructions:\n"
                    "- Answer the question directly and concisely (1-2 sentences)\n"
                    "- Use the KNOWLEDGE BASE for facts when available\n"
                    "- For calendar questions, only mention events on or after today's date\n"
                    "- Don't make up information\n\n"
                    f"Question: {question_text}\n\n"
                    "Answer:"
                )

            else:
                # CONVERSATION: General chat - simplified prompt
                temperature = 0.7
                mode = "CONVERSATION"

                # Get last few exchanges for context (not the whole history)
                recent_lines = conversation_content.strip().split('\n')[-10:]  # Last 10 lines
                recent_context = '\n'.join(recent_lines)

                prompt = (
                    "You are ROSIE, a warm and empathetic conversational companion.\n\n"
                    "Guidelines:\n"
                    "- Respond naturally like a supportive friend would\n"
                    "- Keep responses brief (1-3 sentences)\n"
                    "- When someone shares feelings or problems, be empathetic and listen\n"
                    "- DO NOT give unsolicited advice or suggestions\n"
                    "- DO NOT suggest activities, exercises, or solutions unless explicitly asked\n"
                    "- Simply acknowledge their feelings and show you understand\n"
                    "- Match the user's tone and energy\n\n"
                    "Examples of empathetic responses:\n"
                    "User: I had a really tough day at work.\n"
                    "ROSIE: Oh no, that sounds really rough. I'm sorry you're dealing with that.\n\n"
                    "User: I'm feeling stressed about my deadline.\n"
                    "ROSIE: That's totally understandable - deadline pressure is the worst. How are you holding up?\n\n"
                    f"Recent conversation:\n{recent_context}\n\n"
                    f"User: {question_text}\n"
                    "ROSIE:"
                )

            # Prompt ready - send to Ollama

            # Debug: Log prompt details
            debug_print(f"[PROMPT] Intent: {intent}, Mode: {mode}, Temperature: {temperature}, Max tokens: {max_tokens}")
            debug_print(f"[PROMPT] Has RAG context: {bool(rag_context.strip())}")
            if rag_context.strip():
                debug_print(f"[PROMPT] RAG context length: {len(rag_context)} chars")
            debug_print(f"[PROMPT] Total prompt length: {len(prompt)} chars")
            # Show first AND last parts of prompt for debugging
            debug_print(f"[PROMPT] First 500 chars:\n{prompt[:500]}...")
            debug_print(f"[PROMPT] Last 300 chars:\n...{prompt[-300:]}")

            # DEBUG: Write full prompt to file for manual testing
            debug_prompt_file = self.rosie_dir / "data" / "last_prompt.txt"
            debug_prompt_file.write_text(prompt)
            debug_print(f"[DEBUG] Full prompt written to: {debug_prompt_file}")

            # Call Ollama API
            # Note: Ollama uses 'num_predict' not 'max_tokens', 'num_ctx' for context window
            response = requests.post(
                self.ollama_url,
                json={
                    'model': self.ollama_model,
                    'prompt': prompt,
                    'temperature': temperature,  # Dynamic temperature based on question type
                    'num_predict': max_tokens,  # Ollama-specific parameter name
                    'num_ctx': 4096,  # Context window size (default is often 2048, increase for longer prompts)
                    'stream': False
                },
                timeout=30  # Increased from 5 to 30 seconds for longer contexts
            )

            # Debug: Log Ollama response details
            debug_print(f"[OLLAMA] Status code: {response.status_code}")

            if response.status_code == 200:
                result = response.json()
                debug_print(f"[OLLAMA] Response keys: {result.keys()}")

                # Log token counts to check for truncation
                prompt_eval_count = result.get('prompt_eval_count', 0)
                eval_count = result.get('eval_count', 0)
                debug_print(f"[OLLAMA] Prompt tokens evaluated: {prompt_eval_count}")
                debug_print(f"[OLLAMA] Response tokens generated: {eval_count}")

                ollama_text = result.get('response', '').strip()
                debug_print(f"[OLLAMA] Response text length: {len(ollama_text)} chars")
                debug_print(f"[OLLAMA] Response text: '{ollama_text}'")

                if ollama_text:
                    # Write to speak.txt
                    self.speak_file.write_text(ollama_text)
                    # Show robot response (unless in test modes)
                    if not self.test_questions_mode and not self.test_knowledge_mode and not self.test_calendar_mode and not self.test_conversation_mode:
                        print(f"← ROSIE: {ollama_text}\n")
                    self._log(f"Ollama response: {ollama_text}")

                    # Skip TTS in test modes (text-only)
                    if not self.test_questions_mode and not self.test_knowledge_mode and not self.test_calendar_mode and not self.test_conversation_mode:
                        # Transition to SPEAKING state
                        self._set_state(ConversationState.SPEAKING)

                        # Trigger speech output
                        threading.Thread(target=self._piper_speak, daemon=True).start()
                else:
                    self._log("Ollama returned empty response")
                    self._set_state(ConversationState.LISTENING)
            else:
                debug_print(f"[OLLAMA] Error: API returned status {response.status_code}")
                self._log(f"Ollama API error: {response.status_code}")
                self._set_state(ConversationState.LISTENING)

        except Exception as e:
            debug_print(f"[OLLAMA] Error: {e}")
            import traceback
            traceback.print_exc()
            self._log(f"Ollama error: {e}")
            self._set_state(ConversationState.LISTENING)

    def _summarize_conversation(self, conversation_text):
        """
        Summarize conversation using Ollama

        Returns summary string or None on failure
        """
        try:
            debug_print(f"[OLLAMA] Creating summary of {len(conversation_text)} chars...")

            summary_prompt = (
                f"Condense this conversation into a shorter script format.\n"
                f"Keep only important facts (names, dates, appointments, key topics).\n"
                f"Remove unimportant exchanges and small talk.\n"
                f"Use the exact same format: 'Human: ...' and 'Robot: ...' lines.\n"
                f"Do NOT add any labels or explanations - just output the condensed script.\n\n"
                f"{conversation_text}\n\n"
                f"Condensed conversation:"
            )

            response = requests.post(
                self.ollama_url,
                json={
                    'model': self.ollama_model,
                    'prompt': summary_prompt,
                    'temperature': 0.7,
                    'num_predict': 500,  # Ollama-specific parameter name
                    'stream': False
                },
                timeout=30
            )

            if response.status_code == 200:
                result = response.json()
                summary = result.get('response', '').strip()
                debug_print(f"[OLLAMA] Summary created: {len(summary)} chars")
                return summary
            else:
                debug_print(f"[OLLAMA] Summarization failed: HTTP {response.status_code}")
                return None

        except Exception as e:
            debug_print(f"[OLLAMA] Summarization error: {e}")
            return None

    def _handle_calendar_creation(self, last_human_statement, conversation_content):
        """
        Handle calendar event creation request

        Uses Ollama to extract event details and writes to queue file.
        Provides optimistic confirmation to user.

        Args:
            last_human_statement: The most recent human statement
            conversation_content: Full conversation history for context
        """
        debug_print(f"[CALENDAR] Detected calendar creation request")

        try:
            # Use Ollama to extract structured event details
            import datetime
            now = datetime.datetime.now()
            current_date_context = (
                f"=== IMPORTANT: CURRENT DATE AND TIME ===\n"
                f"RIGHT NOW it is: {now.strftime('%A, %B %d, %Y at %I:%M %p')}\n"
                f"Day of week: {now.strftime('%A')}\n"
                f"Date: {now.strftime('%B %d, %Y')}\n"
                f"Time: {now.strftime('%I:%M %p')}\n"
                f"=== USE THIS DATE/TIME FOR EVENT SCHEDULING ===\n"
            )

            extraction_prompt = (
                f"{current_date_context}\n\n"
                f"CONVERSATION (contains past messages with timestamps):\n{conversation_content}\n\n"
                f"The user wants to create a calendar event. Extract these details from the most recent request:\n"
                f"- summary: Brief title for the event\n"
                f"- date: Date (e.g., 'today', 'tomorrow', 'Friday', 'YYYY-MM-DD')\n"
                f"- time: Time (e.g., '2pm', '14:30', '2:30 PM') or 'none' if not specified\n"
                f"- duration_minutes: Duration in minutes (default 60 if not specified)\n"
                f"- location: Location or 'none' if not specified\n\n"
                f"Respond ONLY with a JSON object in this exact format:\n"
                f'{{"summary": "...", "date": "...", "time": "...", "duration_minutes": 60, "location": "..."}}\n\n'
                f"JSON response:"
            )

            debug_print(f"[CALENDAR] Extracting event details with Ollama...")

            response = requests.post(
                self.ollama_url,
                json={
                    'model': self.ollama_model,
                    'prompt': extraction_prompt,
                    'temperature': 0.1,  # Low temperature for accurate extraction
                    'num_predict': 200,  # Ollama-specific parameter name
                    'stream': False
                },
                timeout=15
            )

            if response.status_code == 200:
                result = response.json()
                ollama_text = result.get('response', '').strip()

                debug_print(f"[CALENDAR] Ollama extraction response: {ollama_text}")

                # Parse JSON from Ollama response
                # Extract JSON from response (might have extra text)
                json_match = re.search(r'\{.*\}', ollama_text, re.DOTALL)
                if json_match:
                    event_data = json.loads(json_match.group(0))

                    # Clean up 'none' values
                    if event_data.get('time') == 'none':
                        event_data['time'] = None
                    if event_data.get('location') == 'none':
                        event_data['location'] = ''

                    # Add timestamp
                    event_data['requested_at'] = now.isoformat()

                    # Load existing queue (in rosie/data/ directory)
                    queue_file = Path(__file__).parent.parent / 'data' / 'calendar_create_queue.json'
                    if queue_file.exists():
                        with open(queue_file, 'r') as f:
                            content = f.read().strip()
                            queue = json.loads(content) if content else []
                    else:
                        queue = []

                    # Add to queue
                    queue.append(event_data)

                    # Save queue
                    with open(queue_file, 'w') as f:
                        json.dump(queue, f, indent=2)

                    debug_print(f"[CALENDAR] ✓ Event queued: {event_data.get('summary')}")

                    # Generate confirmation message
                    confirmation = f"I've queued your {event_data.get('summary')} for {event_data.get('date')}"
                    if event_data.get('time'):
                        confirmation += f" at {event_data.get('time')}"
                    confirmation += ". It will be added to your calendar shortly."

                    # Write to speak.txt
                    self.speak_file.write_text(confirmation)
                    debug_print(f"[CALENDAR] Robot will say: {confirmation}")

                    # Transition to SPEAKING state
                    self._set_state(ConversationState.SPEAKING)

                    # Trigger speech output
                    threading.Thread(target=self._piper_speak, daemon=True).start()

                else:
                    debug_print(f"[CALENDAR] Error: Could not parse JSON from Ollama response")
                    self._speak_error("I understood you want to create an event, but I couldn't extract the details. Please try again.")
            else:
                debug_print(f"[CALENDAR] Error: Ollama API returned status {response.status_code}")
                self._speak_error("I had trouble processing your calendar request. Please try again.")

        except Exception as e:
            debug_print(f"[CALENDAR] Error: {e}")
            import traceback
            traceback.print_exc()
            self._speak_error("I encountered an error processing your calendar request.")

    def _speak_error(self, message):
        """Speak an error message and return to listening"""
        self.speak_file.write_text(message)
        self._set_state(ConversationState.SPEAKING)
        threading.Thread(target=self._piper_speak, daemon=True).start()

    # =====================================================================
    # STATE 3: SPEAKING - Piper TTS Output
    # =====================================================================

    def _speak_immediately(self, text):
        """
        Speak text immediately without changing state (for system messages)

        Used for "Let me think" message during summarization.
        """
        try:
            # Update web status to show "thinking" during this message
            self._update_web_status(ConversationState.RESPONDING, custom_message=text)

            import subprocess
            # Escape quotes and special characters for shell safety
            text_escaped = text.replace('"', '\\"').replace('$', '\\$').replace('`', '\\`')
            piper_cmd = f'echo "{text_escaped}" | piper --model {self.piper_model_path} --output-raw | aplay -r 22050 -f S16_LE -t raw -'
            subprocess.run(piper_cmd, shell=True, check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            debug_print(f"[PIPER] {text}")
        except Exception as e:
            self._log(f"Piper immediate speech error: {e}")

    def _piper_speak(self):
        """
        Piper text-to-speech output

        Converts speak.txt to speech and routes audio based on audio mode:
        - LOCAL mode: Play through system speakers via aplay
        - WEB mode: Send audio to browser via WebSocket

        Supports interrupts via self.tts_interrupt_event
        """
        self._log("Piper TTS started")

        # Skip audio output in text-only mode
        if self.text_only_mode:
            self._log("Text-only mode: Skipping Piper TTS")
            self._set_state(ConversationState.LISTENING)
            return

        try:
            # Read speak.txt
            text = self.speak_file.read_text().strip()

            if not text:
                self._log("speak.txt is empty, skipping TTS")
                self._set_state(ConversationState.LISTENING)
                return

            # Generate audio with Piper
            import subprocess
            import io
            import wave

            # Escape quotes and special characters for shell safety
            text_escaped = text.replace('"', '\\"').replace('$', '\\$').replace('`', '\\`')

            # Check audio mode
            audio_mode = self._get_audio_mode()
            interrupted = False

            if audio_mode == AudioMode.LOCAL:
                # LOCAL mode: Use aplay pipeline with interrupt support
                piper_cmd = f'echo "{text_escaped}" | piper --model {self.piper_model_path} --output-raw | aplay -r 22050 -f S16_LE -t raw -'

                # Start TTS process (non-blocking)
                with self.tts_process_lock:
                    self.tts_process = subprocess.Popen(
                        piper_cmd,
                        shell=True,
                        stdout=subprocess.DEVNULL,
                        stderr=subprocess.DEVNULL
                    )

                # Monitor for interrupts
                while self.tts_process.poll() is None:  # Process still running
                    if self.tts_interrupt_event.is_set():
                        debug_print("[INTERRUPT] Stopping TTS playback (LOCAL mode)...")
                        self.tts_process.terminate()
                        time.sleep(0.1)
                        if self.tts_process.poll() is None:
                            self.tts_process.kill()
                        self.tts_interrupt_event.clear()
                        interrupted = True
                        break
                    time.sleep(0.05)  # Check every 50ms

                # Cleanup
                with self.tts_process_lock:
                    self.tts_process = None

                if not interrupted:
                    self._log(f"Piper spoke (LOCAL): {text}")

            else:  # WEB mode
                # Capture Piper output to buffer
                piper_cmd = f'echo "{text_escaped}" | piper --model {self.piper_model_path} --output-raw'
                result = subprocess.run(piper_cmd, shell=True, check=True, capture_output=True)
                raw_audio = result.stdout

                # Convert raw PCM to WAV format for browser
                wav_buffer = io.BytesIO()
                with wave.open(wav_buffer, 'wb') as wav_file:
                    wav_file.setnchannels(1)  # Mono
                    wav_file.setsampwidth(2)  # 16-bit
                    wav_file.setframerate(22050)  # 22.05 kHz
                    wav_file.writeframes(raw_audio)

                wav_data = wav_buffer.getvalue()

                # Calculate audio duration for animation sync
                # raw_audio is PCM: sample_rate=22050, 16-bit (2 bytes), mono
                audio_duration = len(raw_audio) / (22050 * 2)  # duration in seconds

                # Send to browser via web server
                if self.web_server_module:
                    success = self.web_server_module.send_audio_output_to_browser(wav_data)
                    if success:
                        self._log(f"Piper spoke (WEB): {text}")

                        # Keep SPEAKING state active while audio plays on browser
                        # Sleep in small increments to allow interrupt checking
                        sleep_time = 0
                        sleep_increment = 0.05  # Check every 50ms

                        while sleep_time < audio_duration:
                            if self.tts_interrupt_event.is_set():
                                debug_print("[INTERRUPT] Stopping TTS playback (WEB mode)...")
                                # Note: Browser will finish playing buffered audio
                                # TODO: Implement browser-side audio stop if needed
                                self.tts_interrupt_event.clear()
                                interrupted = True
                                break
                            time.sleep(sleep_increment)
                            sleep_time += sleep_increment
                    else:
                        debug_print(f"[TTS] Failed to send audio to browser")
                        self._log(f"Failed to send audio to browser, no clients connected")
                else:
                    debug_print(f"[TTS] Web server module not available")
                    self._log(f"Web server module not available, cannot send audio")

            # Only update history if not interrupted
            if not interrupted:
                # Append to conversation history with "Robot:" prefix
                self._append_to_history(f"Robot: {text}\n")

            # Clear speak.txt
            self.speak_file.write_text('')

            # Transition back to LISTENING state
            self._set_state(ConversationState.LISTENING)

        except Exception as e:
            self._log(f"Piper error: {e}")
            import traceback
            traceback.print_exc()
            self._set_state(ConversationState.LISTENING)
            # Cleanup on error
            with self.tts_process_lock:
                if self.tts_process:
                    try:
                        self.tts_process.terminate()
                    except:
                        pass
                    self.tts_process = None


    # =====================================================================
    # System Control
    # =====================================================================

    def start(self):
        """Start ROSIE system (all threads)"""
        self._log("Starting ROSIE Conversational AI System...")
        debug_print("[DEBUG] start() called", flush=True)

        # Test questions mode: Run comprehension test (text-only, no audio)
        if self.test_questions_mode:
            self._run_test_questions()
            self.shutdown()
            return

        # Test knowledge mode: Run RAG retrieval test (text-only, no audio)
        if self.test_knowledge_mode:
            self._run_test_knowledge()
            self.shutdown()
            return

        # Test calendar mode: Run calendar accuracy test (text-only, no audio)
        if self.test_calendar_mode:
            self._run_test_calendar()
            self.shutdown()
            return

        # Test conversation mode: Run conversation quality test (text-only, no audio)
        if self.test_conversation_mode:
            self._run_test_conversation()
            self.shutdown()
            return

        # Text-only mode: Skip all audio processing, use stdin/stdout
        if self.text_only_mode:
            print("\n" + "="*70)
            print("ROSIE TEXT-ONLY MODE")
            print("="*70)
            print("All audio processing bypassed (no Whisper/Piper)")
            print("Using text input/output only")
            print("="*70)
            print("\n" + "="*70)
            print("✓ ROSIE IS READY")
            print("="*70)
            print("Type your messages and press Enter. Press CTRL+C to exit.")
            print("="*70 + "\n")

            try:
                while not self.shutdown_event.is_set():
                    try:
                        user_input = input("You: ").strip()
                        if not user_input:
                            continue

                        # Add timestamp like normal conversation flow
                        import datetime
                        timestamp = datetime.datetime.now().strftime('%I:%M %p')

                        # Append to conversation history (like Whisper does)
                        history_entry = f"Human [{timestamp}]: {user_input}\n"
                        with open(self.history_file, 'a') as f:
                            f.write(history_entry)

                        # Generate response using Ollama
                        print("\nROSIE: ", end='', flush=True)
                        self._ollama_response()

                        # Read response from speak file
                        response = self.speak_file.read_text().strip()
                        print(response)
                        print()  # Blank line for readability

                        # Append robot response to history
                        timestamp = datetime.datetime.now().strftime('%I:%M %p')
                        history_entry = f"Robot [{timestamp}]: {response}\n"
                        with open(self.history_file, 'a') as f:
                            f.write(history_entry)

                    except EOFError:
                        print("\nEOF received, exiting...")
                        break
                    except Exception as e:
                        print(f"\nError: {e}")
                        import traceback
                        traceback.print_exc()

            except KeyboardInterrupt:
                print("\nExiting text-only mode...")
            finally:
                self.shutdown()
            return

        # Test mode: Start audio threads for full pipeline testing
        if self.test_mode:
            print("\n" + "="*70)
            print("ROSIE TEST MODE")
            print("="*70)
            print("Full audio pipeline test - Testing: Piper → Speakers → Microphone → Whisper")
            print("="*70 + "\n")

            # In test mode, we NEED the audio threads running to capture Piper output
            # So we start them like normal mode, THEN inject test audio

            # Disable web server for test mode (LOCAL audio only)
            self._disable_web_audio_status()

            # Start Whisper worker
            debug_print("[TEST] Starting Whisper thread for audio capture...", flush=True)
            self.whisper_thread = threading.Thread(target=self._whisper_worker, daemon=True)
            self.whisper_thread.start()

            # Start wake word detector
            debug_print("[TEST] Starting wake word thread...", flush=True)
            self.wake_word_thread = threading.Thread(target=self._wake_word_detector, daemon=True)
            self.wake_word_thread.start()

            # Wait for threads to initialize
            debug_print("[TEST] Waiting for audio system to initialize...", flush=True)
            time.sleep(2)  # Give Whisper time to load model and start audio stream

            debug_print("[TEST] Audio system ready!\n", flush=True)

            if self.test_input:
                # Single test input provided
                debug_print(f"[TEST] Injecting text: \"{self.test_input}\"")
                self._process_test_input(self.test_input)
                # Shutdown immediately after test completes
                self.shutdown()
            else:
                # Interactive test mode
                print("Interactive test mode. Type your messages (include 'Rosie' for responses).")
                print("Press CTRL+C to exit.\n")
                try:
                    while not self.shutdown_event.is_set():
                        user_input = input("You: ").strip()
                        if user_input:
                            self._process_test_input(user_input)
                except (KeyboardInterrupt, EOFError):
                    print("\nExiting test mode...")
                    self.shutdown()
            return

        # Normal mode: Start all audio processing threads
        # Try to import web server module for audio integration
        # Check if web server is actually running first
        debug_print("[DEBUG] Checking for web server...", flush=True)

        web_server_detected = False

        # Try both HTTPS and HTTP (web server may use self-signed cert)
        for protocol in ['https', 'http']:
            try:
                import requests
                import urllib3
                # Disable SSL warnings for self-signed certificates
                urllib3.disable_warnings(urllib3.exceptions.InsecureRequestWarning)

                url = f'{protocol}://localhost:5000/api/state'
                debug_print(f"[DEBUG] Trying {url}...", flush=True)
                response = requests.get(url, timeout=2.0, verify=False)
                debug_print(f"[DEBUG] Web server responded: {response.status_code} via {protocol.upper()}", flush=True)
                web_server_detected = True
                break
            except Exception as e:
                debug_print(f"[DEBUG] {protocol.upper()} failed: {type(e).__name__}", flush=True)
                continue

        if web_server_detected:
            # Web server is running, import module
            if self._import_web_server_module():
                debug_print(f"\n{'='*70}")
                debug_print(f"WEB SERVER DETECTED - Web audio support ENABLED")
                debug_print(f"Remote browsers can enable web audio for I/O")
                debug_print(f"{'='*70}\n", flush=True)
                self._log("Web server detected, enabling web audio support")
            else:
                debug_print(f"\n{'='*70}")
                debug_print(f"WARNING: Web server detected but module import FAILED")
                debug_print(f"Web audio will NOT be available")
                debug_print(f"{'='*70}\n", flush=True)
        else:
            # Web server not running, disable web audio status
            debug_print(f"\n{'='*70}")
            debug_print(f"Web server NOT detected (LOCAL audio mode only)")
            debug_print(f"Web audio will NOT be available")
            debug_print(f"{'='*70}\n", flush=True)
            self._disable_web_audio_status()
            self._log("Web server not detected, using LOCAL audio mode only")

        debug_print("[DEBUG] Web server check complete", flush=True)

        # Load Whisper model (step 4 of 4) - do this synchronously to show progress
        self._current_init_step += 1
        print(f"(Initialization step {self._current_init_step} of {self._total_init_steps}) Loading speech recognition model...", flush=True)
        self._load_whisper_model()

        # Start Whisper worker (model already loaded)
        debug_print("[DEBUG] Starting Whisper thread...", flush=True)
        self.whisper_thread = threading.Thread(target=self._whisper_worker, daemon=True)
        self.whisper_thread.start()
        debug_print("[DEBUG] Whisper thread started", flush=True)

        # Start wake word detector
        debug_print("[DEBUG] Starting wake word thread...", flush=True)
        self.wake_word_thread = threading.Thread(target=self._wake_word_detector, daemon=True)
        self.wake_word_thread.start()
        debug_print("[DEBUG] Wake word thread started", flush=True)

        # Start web audio poll worker (monitors for web audio mode switching)
        debug_print("[DEBUG] Starting web audio poll thread...", flush=True)
        self.web_audio_thread = threading.Thread(target=self._web_audio_poll_worker, daemon=True)
        self.web_audio_thread.start()
        debug_print("[DEBUG] Web audio poll thread started", flush=True)

        # Start keyboard listener (spacebar = wake word)
        debug_print("[DEBUG] Starting keyboard listener thread...", flush=True)
        self.keyboard_thread = threading.Thread(target=self._keyboard_listener, daemon=True)
        self.keyboard_thread.start()
        debug_print("[DEBUG] Keyboard listener thread started", flush=True)

        # Check Ollama GPU status (skip journalctl - can cause hangs)
        ollama_gpu_status = "Active"

        # Get hostname for web URL
        debug_print("[DEBUG] Getting hostname...", flush=True)
        import socket
        try:
            hostname = socket.gethostname()
        except:
            hostname = "localhost"
        debug_print(f"[DEBUG] Hostname: {hostname}", flush=True)

        # Check if web server is using HTTPS (SSL certificate exists)
        debug_print("[DEBUG] Checking SSL cert...", flush=True)
        ssl_cert_path = self.rosie_dir / 'data' / 'ssl' / 'rosie.crt'
        protocol = "https" if ssl_cert_path.exists() else "http"
        debug_print(f"[DEBUG] Protocol: {protocol}", flush=True)

        # Always print startup message and instructions - make it prominent!
        print("\n" + "="*70, flush=True)
        print("✓ ROSIE IS READY", flush=True)
        print("="*70, flush=True)
        print("Say 'Rosie' or press SPACEBAR to start talking.", flush=True)
        print("Press CTRL+C to exit.", flush=True)
        print("="*70 + "\n", flush=True)

        # Debug info (only shown with --debug)
        debug_print(f"Current State: {self._get_state().name}", flush=True)
        debug_print(f"Audio Mode: {self._get_audio_mode().name}", flush=True)
        debug_print(f"LLM (Ollama): {ollama_gpu_status}", flush=True)

        # Show web interface URL if web server is available
        debug_print("[DEBUG] Checking web server module...", flush=True)
        if self.web_server_module:
            debug_print(f"\n🌐 Web Interface: {protocol}://{hostname}:5000", flush=True)
            debug_print(f"   (or use {protocol}://localhost:5000 on this machine)", flush=True)
            if protocol == "https":
                debug_print(f"   (Accept security warning for self-signed certificate)", flush=True)

        debug_print("[DEBUG] Printing final instructions...", flush=True)
        debug_print("\nSpeak naturally - everything is transcribed.", flush=True)
        debug_print("Say 'Rosie' to get a response.", flush=True)
        debug_print("Say 'Rosie, forget everything' to reset conversation history.", flush=True)
        debug_print("Press CTRL+C to exit.", flush=True)
        debug_print("="*70 + "\n", flush=True)
        debug_print("[DEBUG] Entering main loop...", flush=True)

        self._log("ROSIE system started. Say 'Rosie' to get a response.")

        # Main loop (keep alive and monitor private mode)
        try:
            while not self.shutdown_event.is_set():
                # Check for private mode timeout (every 0.5 seconds)
                self._check_private_timeout()

                # Check for private mode change signal from web interface
                if self.private_mode_change_file.exists():
                    try:
                        # Read change request
                        with open(self.private_mode_change_file, 'r') as f:
                            change_request = json.load(f)

                        new_enabled = change_request.get('enabled', False)
                        new_authenticated = change_request.get('authenticated', False)

                        debug_print(f"\n[PRIVATE MODE] Change request: enabled={new_enabled}, authenticated={new_authenticated}")

                        # Update state
                        with self.private_mode_lock:
                            old_enabled = self.private_mode_enabled
                            self.private_mode_enabled = new_enabled
                            self.private_mode_authenticated = new_authenticated
                            self.last_activity_timestamp = time.time()

                        # Reload RAG if mode changed
                        if old_enabled != new_enabled and self.rag_system:
                            debug_print(f"[PRIVATE MODE] Reloading RAG system...")
                            success = self.rag_system.reload_with_private_mode(new_enabled)
                            if success:
                                debug_print(f"[PRIVATE MODE] RAG reloaded successfully")
                            else:
                                debug_print(f"[PRIVATE MODE] Warning: RAG reload failed")

                        # Update web status
                        self._update_web_status(self._get_state())

                        # Delete signal file
                        self.private_mode_change_file.unlink()
                        debug_print(f"[PRIVATE MODE] Change applied and signal file deleted")

                    except Exception as e:
                        debug_print(f"[PRIVATE MODE] Error processing change request: {e}")
                        # Try to delete file anyway
                        try:
                            self.private_mode_change_file.unlink()
                        except:
                            pass

                time.sleep(0.5)
        except KeyboardInterrupt:
            self.shutdown()

    def shutdown(self):
        """Graceful shutdown with SIGINT handling"""
        debug_print("\n[SHUTDOWN] Stopping ROSIE system...")
        self._log("Shutting down ROSIE system...")

        # Signal all threads to stop
        self.shutdown_event.set()

        # Stop audio stream first (critical for clean shutdown)
        try:
            if self.audio_stream is not None:
                debug_print("[SHUTDOWN] Stopping audio stream...")
                self._stop_audio_stream()
                debug_print("[SHUTDOWN] Audio stream stopped")
        except Exception as e:
            debug_print(f"[SHUTDOWN] Error stopping audio stream: {e}")

        # Wait for threads to finish (with timeout)
        debug_print("[SHUTDOWN] Waiting for threads to finish...")
        threads = [self.whisper_thread, self.wake_word_thread]
        for thread in threads:
            if thread and thread.is_alive():
                thread.join(timeout=2)

        debug_print("[SHUTDOWN] ROSIE system stopped cleanly")
        self._log("ROSIE system stopped")
        sys.exit(0)


def signal_handler(sig, frame):
    """Handle CTRL+C for graceful shutdown"""
    print("\nShutting down...")
    if hasattr(signal_handler, 'rosie_instance'):
        signal_handler.rosie_instance.shutdown()
    else:
        sys.exit(0)


def main():
    """Main entry point"""
    # Parse command-line arguments first to get debug mode
    parser = argparse.ArgumentParser(
        description='ROSIE Conversational AI System - Fully Local Voice Assistant',
        epilog='Examples:\n  ./rosie_conversation.py\n  ./rosie_conversation.py --test-audio "Rosie, what is the weather?"\n  ./rosie_conversation.py --text-only',
        formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument('--test-audio', metavar='TEXT', nargs='?', const='',
                       help='Test mode: Full audio pipeline test (Piper → Speakers → Microphone → Whisper). '
                            'Provide text or leave empty for interactive prompts.')
    parser.add_argument('--text-only', action='store_true',
                       help='Text-only mode: Bypass all audio processing (no Whisper/Piper). '
                            'Uses stdin for input and stdout for output.')
    parser.add_argument('--debug', action='store_true',
                       help='Debug mode: Show verbose output including RAG queries, Ollama calls, '
                            'VAD status, and audio processing details. Default output shows only '
                            'transcriptions and wake word detection.')
    parser.add_argument('--test-questions', action='store_true',
                       help='Test mode: Verify ROSIE comprehension by providing a paragraph and '
                            'asking a question with a known answer. Uses text-only mode.')
    parser.add_argument('--test-knowledge', action='store_true',
                       help='Test mode: Verify RAG retrieval by asking a question about data '
                            'in the knowledge_base folder. Uses text-only mode.')
    parser.add_argument('--test-calendar', action='store_true',
                       help='Test mode: Verify Google Calendar accuracy by asking about events '
                            'happening today, in the past, or in the future. Uses text-only mode.')
    parser.add_argument('--test-conversation', action='store_true',
                       help='Test mode: Verify conversation quality by testing intent classification '
                            'and response appropriateness for greetings, acknowledgments, etc.')
    args = parser.parse_args()

    # Set global debug mode
    global DEBUG_MODE
    DEBUG_MODE = args.debug

    # Configure logging based on debug mode
    configure_logging(DEBUG_MODE)

    # Now print startup messages (respects debug mode)
    debug_print("="*70)
    debug_print("ROSIE Conversational AI System - Starting up...")
    debug_print("="*70)

    # Register signal handler for graceful shutdown
    signal.signal(signal.SIGINT, signal_handler)

    # Create ROSIE instance
    debug_print("\n[INIT] Creating ROSIE instance...")
    rosie = RosieConversation(
        test_mode=args.test_audio is not None,
        test_input=args.test_audio,
        text_only_mode=args.text_only,
        test_questions_mode=args.test_questions,
        test_knowledge_mode=args.test_knowledge,
        test_calendar_mode=args.test_calendar,
        test_conversation_mode=args.test_conversation
    )
    debug_print("[INIT] ROSIE instance created successfully")

    # Store instance for signal handler
    signal_handler.rosie_instance = rosie

    # Start system
    rosie.start()


if __name__ == '__main__':
    main()
