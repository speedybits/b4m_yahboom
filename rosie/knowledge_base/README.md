# ROSIE Knowledge Base

This directory contains markdown files that form ROSIE's RAG (Retrieval-Augmented Generation) knowledge base.

## Purpose

ROSIE uses these markdown documents to:
- Answer questions about personal information
- Provide context-aware responses
- Access calendar events and schedules
- Reference family information, health data, and other personal details

## How It Works

1. All `.md` files in this directory are automatically indexed by ROSIE
2. ChromaDB creates vector embeddings stored in `../data/.rosie_vector_db/`
3. When you ask ROSIE a question, it searches these documents for relevant information
4. The retrieved context is provided to the LLM for accurate answers

## File Examples

Common knowledge base files:
- `calendar_events.md` - Synced Google Calendar events (auto-generated)
- `family.md` - Family member information
- `pronunciations.md` - Custom pronunciation guides
- `*.md` - Any other personal information documents

## Privacy

**All `.md` files in this directory are excluded from git** to protect your privacy.
Only this README is committed to preserve the directory structure.

## Adding New Documents

Simply create new `.md` files in this directory with your information.
ROSIE will automatically index them on next startup.

Format tips:
- Use clear headings with `#` markdown syntax
- Write in natural language
- Include dates, names, and specific details
- Organize related information together
