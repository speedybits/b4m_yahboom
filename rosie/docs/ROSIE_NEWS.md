# ROSIE Daily News Integration

## Overview

This document outlines the proposed system for integrating daily news updates into ROSIE's knowledge base using NPR's RSS feeds.

## Goal

Enable ROSIE to have access to current news by automatically fetching and storing news articles daily in the knowledge base, making them available for RAG (Retrieval-Augmented Generation) queries.

## Legal Compliance

### Why NOT text.npr.org Scraping

**CRITICAL**: text.npr.org explicitly prohibits all automated scraping in robots.txt:
```
User-agent: *
Disallow: /
```

Scraping would:
- ❌ Violate robots.txt protocol
- ❌ Potentially violate NPR's Terms of Service
- ❌ Risk IP blocking or legal action
- ❌ Raise ethical concerns

### Recommended Approach: RSS Feeds

**Use NPR's Official RSS Feeds Instead:**
- ✅ Legal and ethical - designed for automated consumption
- ✅ Cleaner, structured data
- ✅ More reliable and maintainable
- ✅ No rate limiting concerns

**Available NPR RSS Feeds:**
- News: `https://feeds.npr.org/1001/rss.xml`
- Politics: `https://feeds.npr.org/1014/rss.xml`
- Technology: `https://feeds.npr.org/1019/rss.xml`
- Science: `https://feeds.npr.org/1007/rss.xml`
- Full list: https://www.npr.org/rss/rss.php

## Current Infrastructure (Already Available)

### Existing RAG System
**Location**: `/home/mike/projects/b4m_yahboom/knowledge_base/`

**Technology Stack:**
- **Framework**: LlamaIndex for document indexing
- **Vector Database**: ChromaDB with persistent storage at `.rosie_vector_db/`
- **Embeddings**: Ollama's `nomic-embed-text` model (local, no API calls)
- **Document Format**: Markdown (.md files)
- **Indexing**: Automatic on startup - scans all .md files

**Key Features:**
- Automatic indexing of all `.md` files in knowledge_base/
- Semantic search using vector embeddings
- Top-K retrieval (returns 3 most relevant chunks per query)
- Persistent storage via ChromaDB
- Hot reload capability (delete `.rosie_vector_db/` to rebuild)

### Available Libraries
- ✅ beautifulsoup4 (4.14.2) - HTML parsing
- ✅ lxml (4.8.0) - Fast XML/HTML processing
- ✅ requests (2.32.5) - HTTP client
- ✅ numpy (>=1.24.0) - Data manipulation

### Required Additions
- ⚠️ feedparser - RSS/Atom feed parsing (not yet installed)
- ⚠️ schedule (optional) - Python job scheduling

### Scheduling Capability
- ✅ Cron available and ready to use
- ✅ No existing crontab configured (clean slate)

## Proposed Implementation

### Architecture Overview

```
┌─────────────────┐
│  NPR RSS Feeds  │
│  (multiple)     │
└────────┬────────┘
         │ Daily @ 6 AM (cron)
         ▼
┌─────────────────┐
│ fetch_npr_news  │
│  .py script     │
└────────┬────────┘
         │ Parse & Convert
         ▼
┌─────────────────┐
│  knowledge_base │
│  /npr_news_     │
│  YYYY-MM-DD.md  │
└────────┬────────┘
         │ Auto-indexed
         ▼
┌─────────────────┐
│  ROSIE RAG      │
│  ChromaDB       │
└─────────────────┘
```

### Workflow

1. **Daily Fetch** (6 AM via cron):
   - Parse NPR RSS feeds (news, politics, technology)
   - Extract articles (title, summary, link, date)
   - Convert to structured markdown format
   - Save as `npr_news_YYYY-MM-DD.md` in knowledge_base/

2. **Auto-Cleanup** (7 AM via cron):
   - Delete news files older than 7 days (configurable)
   - Keeps storage manageable (rolling 1-week window)

3. **Vector DB Rebuild** (7 AM via cron):
   - Delete `.rosie_vector_db/` to force fresh indexing
   - ROSIE rebuilds on next startup with new news included
   - Alternative: Implement hot reload for immediate availability

4. **RAG Integration**:
   - ROSIE automatically queries knowledge base for relevant context
   - News articles become part of conversational context
   - No code changes needed in ROSIE conversation logic

### File Structure

```
/home/mike/projects/b4m_yahboom/
├── knowledge_base/
│   ├── family.md              (existing)
│   ├── rosie.md               (existing)
│   ├── yardcare.md            (existing)
│   └── npr_news_2025-11-10.md (new, daily)
│
├── scripts/
│   ├── fetch_npr_news.py      (new)
│   ├── cleanup_old_news.py    (new)
│   └── daily_news_update.sh   (new, orchestrator)
│
└── .rosie_vector_db/          (rebuilt daily)
```

## Implementation Details

### Script 1: fetch_npr_news.py

**Purpose**: Fetch NPR RSS feeds and convert to markdown

**Pseudocode:**
```python
import feedparser
from datetime import datetime
from pathlib import Path

def fetch_npr_news(feeds, output_dir):
    """
    Args:
        feeds: dict of {category: rss_url}
        output_dir: Path to knowledge_base directory

    Returns:
        Path to created markdown file
    """
    date_str = datetime.now().strftime("%Y-%m-%d")
    output_file = Path(output_dir) / f"npr_news_{date_str}.md"

    with open(output_file, 'w') as f:
        f.write(f"# NPR News - {date_str}\n\n")

        for category, feed_url in feeds.items():
            feed = feedparser.parse(feed_url)
            f.write(f"## {category.title()}\n\n")

            # Get top 5-10 articles per category
            for entry in feed.entries[:5]:
                f.write(f"### {entry.title}\n")
                f.write(f"*Published: {entry.published}*\n\n")
                f.write(f"{entry.summary}\n\n")
                f.write(f"[Read more]({entry.link})\n\n")
                f.write("---\n\n")

    return output_file

# Configuration
FEEDS = {
    "news": "https://feeds.npr.org/1001/rss.xml",
    "politics": "https://feeds.npr.org/1014/rss.xml",
    "technology": "https://feeds.npr.org/1019/rss.xml",
}

KB_DIR = "/home/mike/projects/b4m_yahboom/knowledge_base"

if __name__ == "__main__":
    output = fetch_npr_news(FEEDS, KB_DIR)
    print(f"✅ News saved to: {output}")
```

### Script 2: cleanup_old_news.py

**Purpose**: Delete news files older than N days

**Pseudocode:**
```python
from pathlib import Path
from datetime import datetime, timedelta

def cleanup_old_news(kb_dir, max_age_days=7):
    """
    Remove news files older than max_age_days

    Args:
        kb_dir: Path to knowledge_base directory
        max_age_days: Keep news files for this many days
    """
    kb_path = Path(kb_dir)
    cutoff_date = datetime.now() - timedelta(days=max_age_days)

    deleted_count = 0
    for news_file in kb_path.glob("npr_news_*.md"):
        # Get file modification time
        file_mtime = datetime.fromtimestamp(news_file.stat().st_mtime)

        if file_mtime < cutoff_date:
            news_file.unlink()
            deleted_count += 1
            print(f"🗑️  Deleted: {news_file.name}")

    print(f"✅ Cleanup complete: {deleted_count} old files removed")

if __name__ == "__main__":
    KB_DIR = "/home/mike/projects/b4m_yahboom/knowledge_base"
    cleanup_old_news(KB_DIR, max_age_days=7)
```

### Script 3: daily_news_update.sh

**Purpose**: Orchestrate daily news update workflow

```bash
#!/bin/bash
# /home/mike/projects/b4m_yahboom/scripts/daily_news_update.sh

# Configuration
SCRIPT_DIR="/home/mike/projects/b4m_yahboom/scripts"
KB_DIR="/home/mike/projects/b4m_yahboom/knowledge_base"
VECTOR_DB="/home/mike/projects/b4m_yahboom/.rosie_vector_db"
PYTHON="/usr/bin/python3"

echo "=== NPR News Update Started: $(date) ==="

# Step 1: Fetch latest news from RSS feeds
echo "📰 Fetching NPR news..."
$PYTHON "$SCRIPT_DIR/fetch_npr_news.py"
if [ $? -eq 0 ]; then
    echo "✅ News fetch successful"
else
    echo "❌ News fetch failed"
    exit 1
fi

# Step 2: Cleanup old news files
echo "🗑️  Cleaning up old news..."
$PYTHON "$SCRIPT_DIR/cleanup_old_news.py"

# Step 3: Rebuild vector database for immediate indexing
# (Alternative: Let ROSIE rebuild on next restart)
echo "🔄 Rebuilding vector database..."
rm -rf "$VECTOR_DB"
echo "✅ Vector DB cleared (will rebuild on next ROSIE start)"

# Step 4: Optional - Restart ROSIE if running
# Uncomment if you want automatic ROSIE restart
# if pgrep -f "rosie_conversation.py" > /dev/null; then
#     echo "🔄 Restarting ROSIE..."
#     pkill -f "rosie_conversation.py"
#     # Add command to restart ROSIE here
# fi

echo "=== NPR News Update Complete: $(date) ==="
```

### Cron Configuration

Add to user crontab (`crontab -e`):

```bash
# NPR News Daily Update
# Runs at 6:00 AM every day
0 6 * * * /home/mike/projects/b4m_yahboom/scripts/daily_news_update.sh >> /tmp/rosie_news_update.log 2>&1

# Alternative: Multiple updates per day (6 AM, 12 PM, 6 PM)
# 0 6,12,18 * * * /home/mike/projects/b4m_yahboom/scripts/daily_news_update.sh >> /tmp/rosie_news_update.log 2>&1
```

**Cron Explanation:**
- `0 6 * * *` - At 6:00 AM daily
- Script output logged to `/tmp/rosie_news_update.log`
- Errors captured via `2>&1`

## Configuration Options

### Adjustable Parameters

**Feed Selection:**
```python
FEEDS = {
    "news": "https://feeds.npr.org/1001/rss.xml",
    "politics": "https://feeds.npr.org/1014/rss.xml",
    "technology": "https://feeds.npr.org/1019/rss.xml",
    "science": "https://feeds.npr.org/1007/rss.xml",
    # Add more as needed
}
```

**Articles Per Category:**
```python
for entry in feed.entries[:5]:  # Change 5 to desired number
```

**News Retention Period:**
```python
cleanup_old_news(KB_DIR, max_age_days=7)  # Keep 1 week
# Options: 3, 7, 14, 30 days
```

**Update Frequency:**
```bash
# Daily at 6 AM
0 6 * * * /path/to/script

# Multiple times per day
0 6,12,18 * * * /path/to/script

# Every 6 hours
0 */6 * * * /path/to/script
```

## Installation Steps

### 1. Install Required Dependencies

```bash
pip install feedparser
```

### 2. Create Scripts Directory

```bash
mkdir -p /home/mike/projects/b4m_yahboom/scripts
```

### 3. Create Python Scripts

Create the three Python scripts as outlined above:
- `scripts/fetch_npr_news.py`
- `scripts/cleanup_old_news.py`
- `scripts/daily_news_update.sh`

### 4. Make Shell Script Executable

```bash
chmod +x /home/mike/projects/b4m_yahboom/scripts/daily_news_update.sh
```

### 5. Test Manual Execution

```bash
# Test news fetch
python3 scripts/fetch_npr_news.py

# Verify markdown file created
ls -lh knowledge_base/npr_news_*.md

# Test full workflow
./scripts/daily_news_update.sh
```

### 6. Verify RAG Integration

```bash
# Restart ROSIE (or delete vector DB to force rebuild)
rm -rf .rosie_vector_db

# Start ROSIE and ask news-related question
# Example: "What's happening in politics today?"
```

### 7. Configure Cron Job

```bash
# Edit crontab
crontab -e

# Add line:
0 6 * * * /home/mike/projects/b4m_yahboom/scripts/daily_news_update.sh >> /tmp/rosie_news_update.log 2>&1

# Verify cron job added
crontab -l
```

### 8. Monitor First Execution

```bash
# Check log file after 6 AM next day
cat /tmp/rosie_news_update.log

# Verify news file created
ls -lh knowledge_base/npr_news_*.md

# Check vector DB rebuilt
ls -lh .rosie_vector_db/
```

## Technical Considerations

### Storage Requirements

**Per Day:**
- Markdown file: ~5-15 KB per category
- Total daily: ~20-50 KB (3-4 categories)
- Weekly retention: ~140-350 KB

**Vector Database:**
- ChromaDB embeddings: ~1-5 MB per 100 markdown files
- Impact: Minimal with 7-day retention

### Performance Impact

**ROSIE Response Time:**
- RAG retrieval adds: 60-100ms
- With news: Minimal additional impact (~5-10ms)
- Total still well under 200ms

**Memory Usage:**
- ChromaDB: ~150-250MB RAM
- With news: +10-20MB (7-day retention)

**Indexing Time:**
- Full rebuild: 5-15 seconds
- Daily incremental: 1-3 seconds

### Network Considerations

**Bandwidth:**
- RSS feed fetch: ~10-50 KB per feed
- Total daily: ~100-200 KB
- Negligible impact

**Reliability:**
- NPR RSS feeds: 99.9%+ uptime
- Graceful failure handling needed

## Potential Issues & Solutions

### Issue 1: RAG Context Pollution

**Problem**: News context drowns out personal data (family, health)

**Solutions:**
1. **Separate Collections** (Advanced):
   - Create `rosie_personal` and `rosie_news` ChromaDB collections
   - Query personal first, news second
   - Weight personal data higher

2. **Prefix Filtering** (Simple):
   - Add metadata to news files
   - Filter by document type during retrieval

3. **Time-based Relevance** (Simple):
   - Recent news more relevant
   - Automatic de-weighting with age

### Issue 2: Vector DB Doesn't Auto-Update

**Problem**: New news files not indexed until ROSIE restart

**Solutions:**
1. **Daily Rebuild** (Current approach):
   - Delete `.rosie_vector_db/` daily
   - ROSIE rebuilds on next start

2. **Hot Reload** (Future enhancement):
   - Modify ROSIE to watch knowledge_base/
   - Auto-reindex on file changes

3. **Scheduled ROSIE Restart**:
   - Restart ROSIE service after news update
   - Ensures immediate availability

### Issue 3: Irrelevant News Articles

**Problem**: Not all NPR news relevant to user

**Solutions:**
1. **Keyword Filtering**:
   - Filter articles by keywords before saving
   - Focus on topics of interest

2. **Summarization** (Advanced):
   - Use Ollama to summarize articles
   - Reduce noise, improve relevance

3. **Manual Feed Selection**:
   - Only subscribe to relevant RSS feeds
   - Skip sports, entertainment, etc.

### Issue 4: RSS Feed Downtime

**Problem**: NPR RSS feed temporarily unavailable

**Solutions:**
1. **Error Handling**:
   - Catch connection errors gracefully
   - Log failure, retry later

2. **Retry Logic**:
   ```python
   for attempt in range(3):
       try:
           feed = feedparser.parse(url)
           break
       except Exception as e:
           if attempt == 2:
               print(f"Failed after 3 attempts: {e}")
   ```

3. **Multiple Sources**:
   - Add BBC, Reuters, AP RSS feeds
   - Fallback if one source fails

## Future Enhancements

### Phase 1: Basic Implementation (Described above)
- RSS feed fetching
- Markdown conversion
- Daily cron job
- Basic cleanup

## Alternative Approaches

## References

### NPR Resources
- RSS Feeds: https://www.npr.org/rss/rss.php
- Terms of Use: https://www.npr.org/about-npr/179876898/terms-of-use
- robots.txt: https://text.npr.org/robots.txt

### Technical Documentation
- feedparser: https://feedparser.readthedocs.io/
- LlamaIndex: https://docs.llamaindex.ai/
- ChromaDB: https://docs.trychroma.com/
- Cron: `man crontab`

### Related Files
- `/home/mike/projects/b4m_yahboom/rosie_conversation.py` - ROSIE RAG implementation
- `/home/mike/projects/b4m_yahboom/knowledge_base/` - Current knowledge base
- `/home/mike/projects/b4m_yahboom/.rosie_vector_db/` - ChromaDB storage

## Status

**Current State**: Proposed (not yet implemented)

**Next Steps**:
1. Review and approve this proposal
2. Install feedparser library
3. Create Python scripts
4. Test manual execution
5. Configure cron job
6. Monitor first automated run

**Estimated Implementation Time**: 2-4 hours

**Estimated Testing Time**: 1-2 days (verify cron, RAG integration)

---

*Document created: 2025-11-10*
*Last updated: 2025-11-10*
