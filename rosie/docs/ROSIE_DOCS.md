# ROSIE Google Docs Integration

Access your Google Drive documents through ROSIE's voice-controlled conversational AI.

## Overview

This integration connects ROSIE to your Google Drive, allowing you to:
- **Access document content**: "Rosie, what does my recipe document say about chocolate cake?"
- **Query multiple documents**: ROSIE searches across all synced documents
- **Support for Google Docs and PDFs**: Syncs both document types automatically
- **Public and private knowledge separation**: `Rosie_Knowledge/Public` for general info, `Rosie_Knowledge/Private` for sensitive data
- **Automatic syncing**: Documents refresh on ROSIE startup and optionally via cron

## Architecture

### RAG-Based Document Access (Fast, No Latency)
- Google Docs and PDFs synced from `Rosie_Knowledge/Public` → `knowledge_base/google_docs/`
- Google Docs and PDFs synced from `Rosie_Knowledge/Private` → `knowledge_base/private/google_docs/`
- ROSIE's existing RAG system indexes documents automatically
- Zero latency during conversation (no API calls)
- Documents update on startup or via scheduled sync
- Private documents excluded from RAG when not in private mode (matches local file behavior)

**Why this approach?**
- Maintains ROSIE's fully-local conversational philosophy
- No external API calls during conversation (no latency/failures)
- Documents searchable alongside other knowledge base content
- Automatic privacy controls matching local knowledge base structure
- Simple hardcoded folder structure (no complex folder selection)

## Setup Guide

### Prerequisites

1. **Python libraries:**
   ```bash
   pip install --upgrade google-api-python-client google-auth-httplib2 google-auth-oauthlib pypdf
   ```

   **Note:** The `pypdf` library is required for PDF text extraction.

2. **Create Google Drive folders:**
   - Go to https://drive.google.com
   - Create folder: `Rosie_Knowledge/Public` (for general documents)
   - Create folder: `Rosie_Knowledge/Private` (for sensitive documents)
   - Place your documents and PDFs in these folders

3. **Google Cloud Project OAuth Credentials:**
   - Go to https://console.cloud.google.com/
   - Create a new project (or use existing)
   - Enable the Google Drive API
   - Configure OAuth consent screen (internal use, your email)
   - Create OAuth 2.0 Desktop credentials
   - Copy Client ID and Client Secret

4. **Add credentials to ~/.bashrc:**
   ```bash
   # Open ~/.bashrc
   nano ~/.bashrc

   # Add these lines at the bottom (replace with your actual values):
   export GOOGLE_DOCS_CLIENT_ID='your-client-id.apps.googleusercontent.com'
   export GOOGLE_DOCS_CLIENT_SECRET='your-client-secret'

   # Save and exit, then reload:
   source ~/.bashrc
   ```

   **Reusing Calendar Credentials:**
   If you already set up Google Calendar integration, you can reuse those credentials.
   The scripts automatically fall back to `GOOGLE_CALENDAR_CLIENT_ID` and `GOOGLE_CALENDAR_CLIENT_SECRET`
   if the Docs-specific variables aren't set.

   **Security Note:** Credentials are stored as environment variables in ~/.bashrc instead of files.
   This follows security best practices and matches ROSIE's existing configuration pattern.

### Initial Setup (One-Time)

1. **Run setup wizard:**
   ```bash
   cd /home/mike/projects/b4m_yahboom
   python3 rosie/src/rosie_docs_setup.py
   ```

   This will:
   - Open browser for one-time Google authentication
   - Search for `Rosie_Knowledge/Public` and `Rosie_Knowledge/Private` folders automatically
   - Save configuration to `rosie/data/docs_config.json`
   - **Generate authentication token and print export command**

   **Note:** If the folders are not found, you'll see an error. Make sure you created
   `Rosie_Knowledge/Public` and `Rosie_Knowledge/Private` folders in your Google Drive first.

2. **Save the generated token to ~/.bashrc:**

   After setup completes, you'll see:
   ```bash
   ================================================================================
   TOKEN GENERATED - SAVE TO ~/.bashrc FOR PERSISTENCE
   ================================================================================

   Add this line to your ~/.bashrc:

   export GOOGLE_DOCS_TOKEN='{"token": "ya29...", "refresh_token": "1//0g...", ...}'
   ```

   Copy that export line and add it to your ~/.bashrc:
   ```bash
   nano ~/.bashrc
   # Paste the GOOGLE_DOCS_TOKEN line at the bottom
   # Save and exit
   source ~/.bashrc
   ```

3. **Test manual sync:**
   ```bash
   python3 rosie/src/rosie_docs_sync.py
   ```

   Verify documents appear in:
   ```bash
   ls rosie/knowledge_base/google_docs/
   cat rosie/knowledge_base/google_docs/[document_name].md
   ```

4. **Set up automatic syncing (optional):**
   ```bash
   crontab -e
   ```

   Add this line:
   ```bash
   # Sync Google Docs every 6 hours
   0 */6 * * * cd /home/mike/projects/b4m_yahboom && python3 rosie/src/rosie_docs_sync.py >> /tmp/rosie_docs_sync.log 2>&1
   ```

   Save and exit. Verify cron job:
   ```bash
   crontab -l
   ```

   **Note:** ROSIE automatically syncs documents on startup, so a cron job is only needed
   if you want documents to update while ROSIE is running for extended periods.

## Usage

### Asking About Documents

Ask ROSIE questions about your synced Google Docs:

```
You: "Rosie, what does my recipe document say about chocolate cake?"
ROSIE: "According to your recipe document, the chocolate cake requires flour, sugar, cocoa powder, eggs, and milk. Bake at 350 degrees for 30 minutes."

You: "Rosie, do I have any notes about the team meeting?"
ROSIE: "Yes, in your meeting notes document it mentions action items: John will review the budget, Sarah will draft the proposal, and the next meeting is scheduled for Friday."

You: "Rosie, what's in my vacation planning document?"
ROSIE: "Your vacation planning document lists destinations including Hawaii in June, a beach resort with hotel booking confirmation, and activities like snorkeling and hiking."
```

**How it works:**
- Public documents stored in `rosie/knowledge_base/google_docs/`
- Private documents stored in `rosie/knowledge_base/private/google_docs/`
- Each document is formatted with markdown header (title, source, timestamp)
- ROSIE's RAG system retrieves relevant content from all documents
- Private documents only included when ROSIE is in private mode
- No API calls during conversation (instant response)

### Configuration

The configuration file `rosie/data/docs_config.json` is created automatically by the setup wizard:

```json
{
  "folders": {
    "public": {
      "name": "Rosie_Knowledge/Public",
      "id": "1abc...xyz",
      "enabled": true
    },
    "private": {
      "name": "Rosie_Knowledge/Private",
      "id": "2def...uvw",
      "enabled": true
    }
  },
  "sync_settings": {
    "file_types": [
      "application/vnd.google-apps.document",
      "application/pdf"
    ]
  }
}
```

**Settings:**
- `folders.public.enabled`: Set to `false` to disable public folder sync
- `folders.private.enabled`: Set to `false` to disable private folder sync
- `file_types`: Supports Google Docs and PDFs (Sheets/Slides not supported)

**Note:** The folder names and structure are hardcoded. You must use `Rosie_Knowledge/Public`
and `Rosie_Knowledge/Private` in your Google Drive.

## File Structure

```
/home/mike/projects/b4m_yahboom/rosie/
├── src/
│   ├── rosie_docs_setup.py             # Setup wizard
│   ├── rosie_docs_sync.py              # Sync documents to RAG
│   └── rosie_conversation.py           # ROSIE (syncs docs on startup)
├── data/
│   └── docs_config.json                # Folder config (NEVER commit)
└── knowledge_base/
    ├── google_docs/                    # Public documents (NEVER commit)
    │   ├── Recipe_Collection.md
    │   ├── Meeting_Notes.md
    │   └── User_Manual.md
    └── private/
        └── google_docs/                # Private documents (NEVER commit)
            ├── Medical_Records.md
            └── Financial_Info.md

# Google Drive structure (you must create these):
# Rosie_Knowledge/Public/              # Syncs to knowledge_base/google_docs/
# Rosie_Knowledge/Private/             # Syncs to knowledge_base/private/google_docs/

# Environment variables in ~/.bashrc (NEVER commit .bashrc!)
# GOOGLE_DOCS_CLIENT_ID or GOOGLE_CALENDAR_CLIENT_ID (fallback)
# GOOGLE_DOCS_CLIENT_SECRET or GOOGLE_CALENDAR_CLIENT_SECRET (fallback)
# GOOGLE_DOCS_TOKEN
```

## PDF Support

ROSIE can sync and read PDF files from your Google Drive alongside Google Docs.

### How PDF Sync Works

1. **Detection**: The sync script finds PDF files in `Rosie_Knowledge/Public` and `Rosie_Knowledge/Private`
2. **Download**: PDFs are downloaded from Google Drive
3. **Text Extraction**: Text content is extracted using the `pypdf` library
4. **Markdown Conversion**: Extracted text is saved as `.md` file with metadata
5. **RAG Indexing**: Text content becomes searchable through ROSIE's RAG system

### PDF Requirements

**Installation:**
```bash
pip install pypdf
```

**Supported PDFs:**
- Text-based PDFs (native digital documents)
- PDFs with embedded text (created from Word, exported from apps, etc.)
- Searchable PDFs (text layer present)

**Unsupported PDFs:**
- Image-only PDFs (scanned documents without OCR)
- Password-protected PDFs
- Heavily formatted PDFs may lose formatting in text extraction

### PDF Examples

```
You: "Rosie, what does the user manual say about installation?"
ROSIE: "According to the user manual PDF, installation requires Python 3.8+, run pip install -r requirements.txt, and execute the setup script."

You: "Rosie, what's in the project specification PDF?"
ROSIE: "The project specification PDF outlines three phases: Phase 1 covers system design, Phase 2 covers implementation, and Phase 3 covers testing and deployment."
```

### Markdown Output Format

PDFs are converted to markdown with metadata:

```markdown
# Document Title (from PDF metadata or filename)

**Source:** https://drive.google.com/file/d/[file-id]/view
**Last Synced:** 2025-12-12 14:30:00
**Type:** PDF

[Extracted text content from PDF]
```

**Note:** Complex PDF formatting (tables, images, multi-column layouts) is converted to plain text.
Some formatting may be lost during extraction.

## Automatic Sync

### On ROSIE Startup
ROSIE automatically syncs Google Docs during initialization:
1. Syncs calendar events (if configured)
2. **Syncs Google Docs** (if configured)
3. Rebuilds RAG index with all knowledge base content
4. Ready for conversation

This ensures ROSIE always has fresh document content on launch.

### Scheduled Sync (Optional)
For long-running ROSIE sessions, add a cron job:

```bash
# Sync every 6 hours
0 */6 * * * cd /home/mike/projects/b4m_yahboom && python3 rosie/src/rosie_docs_sync.py >> /tmp/rosie_docs_sync.log 2>&1
```

**When to use scheduled sync:**
- ROSIE runs continuously for days/weeks
- Documents update frequently throughout the day
- Need real-time document content without restarting ROSIE

**When startup sync is sufficient:**
- ROSIE restarts regularly (daily or more often)
- Documents don't change frequently
- Can restart ROSIE when documents update

## Troubleshooting

### Authentication Issues

**Error: "Google Docs credentials not found in environment variables!"**
- Make sure you added GOOGLE_DOCS_CLIENT_ID and GOOGLE_DOCS_CLIENT_SECRET to ~/.bashrc
- Or set up GOOGLE_CALENDAR_CLIENT_ID/SECRET (scripts use these as fallback)
- Run: `source ~/.bashrc` to load them
- Verify: `echo $GOOGLE_DOCS_CLIENT_ID`

**Error: "GOOGLE_DOCS_TOKEN not found in environment!"**
- Run `rosie/src/rosie_docs_setup.py` to generate token
- Copy the export command it prints
- Add to ~/.bashrc and run `source ~/.bashrc`

**Token refresh messages**
- If you see "Token was refreshed. Update your ~/.bashrc with:", copy the new export line
- This happens when the access token expires (every ~1 hour)
- The refresh token automatically gets a new access token
- Update ~/.bashrc with the new token to persist it

### Documents Not Syncing

**Check if sync is working:**
```bash
# View sync logs (if using cron)
tail -f /tmp/rosie_docs_sync.log

# Manually run sync to see errors
python3 rosie/src/rosie_docs_sync.py

# Check synced documents
ls rosie/knowledge_base/google_docs/
ls rosie/knowledge_base/private/google_docs/
```

**Common issues:**
- **Folders not found**: Make sure you created `Rosie_Knowledge/Public` and `Rosie_Knowledge/Private` in Google Drive
- **No configuration**: Run `rosie/src/rosie_docs_setup.py` to create `docs_config.json`
- **Token expired**: Re-run `rosie/src/rosie_docs_setup.py`
- **Folder not enabled**: Check `enabled: true` in `docs_config.json` for both public and private
- **Permission denied**: Ensure you have view access to the folders

### PDFs Not Syncing

**Check PDF support:**
```bash
# Verify pypdf is installed
python3 -c "import pypdf; print('pypdf installed')"

# If not installed:
pip install pypdf

# Check file_types in config includes PDFs
cat rosie/data/docs_config.json | grep -A 5 file_types
```

**Common PDF issues:**
- **PDFs not appearing**: Verify they're in `Rosie_Knowledge/Public` or `Rosie_Knowledge/Private` in Google Drive
- **Empty content**: PDF may be image-only (no extractable text) - use OCR tools to add text layer
- **pypdf not installed**: Run `pip install pypdf`
- **Password-protected PDFs**: Not supported - remove password protection first

### Private Documents Appearing When They Shouldn't

**ROSIE showing private content in normal mode:**
- Verify ROSIE's private mode is disabled (default)
- Private documents in `knowledge_base/private/google_docs/` should be excluded from RAG
- Check RAG initialization excludes `**/private/*` pattern when not in private mode
- Restart ROSIE to rebuild RAG index with correct privacy settings

### ROSIE Not Finding Document Content

**ROSIE has documents but can't answer questions:**
1. Verify documents were synced: `ls rosie/knowledge_base/google_docs/`
2. Check document content: `cat rosie/knowledge_base/google_docs/[filename].md`
3. Restart ROSIE to rebuild RAG index with new documents
4. Try more specific questions referencing document titles or unique terms

**Empty or missing documents:**
- Check if documents are in `Rosie_Knowledge/Public` or `Rosie_Knowledge/Private` folders
- Verify folder IDs in `docs_config.json` match Google Drive
- Ensure documents are Google Docs or PDFs (Sheets/Slides not supported)
- For PDFs, verify text content can be extracted (not image-only scans)

### Folders Not Found During Setup

**Error: "Could not find Rosie_Knowledge/Public or Rosie_Knowledge/Private folders"**

1. **Create the folders in Google Drive:**
   - Go to https://drive.google.com
   - Create a new folder named exactly `Rosie_Knowledge`
   - Inside it, create two subfolders: `Public` and `Private`
   - Full paths: `Rosie_Knowledge/Public` and `Rosie_Knowledge/Private`

2. **Verify folder names are exact:**
   - Names are case-sensitive: must be `Rosie_Knowledge`, not `rosie_knowledge`
   - Must include the slash: `Rosie_Knowledge/Public`, not `Rosie Knowledge Public`

3. **Re-run setup after creating folders:**
   ```bash
   python3 rosie/src/rosie_docs_setup.py
   ```

### Document Changes Not Reflected in ROSIE

**Documents updated but ROSIE gives old information:**
1. Run manual sync: `python3 rosie/src/rosie_docs_sync.py`
2. Restart ROSIE to rebuild RAG index
3. Verify updated content in `rosie/knowledge_base/google_docs/[filename].md`

**Automatic sync not working:**
- Check cron job exists: `crontab -l`
- View cron logs: `tail -f /tmp/rosie_docs_sync.log`
- ROSIE must restart to load updated documents (RAG index rebuilds on startup)

## Advanced Topics

### Folder Organization

The sync automatically includes all subfolders within `Rosie_Knowledge/Public` and `Rosie_Knowledge/Private`:

**Example folder structure in Google Drive:**
```
Rosie_Knowledge/
├── Public/
│   ├── Recipes/
│   │   ├── Desserts.gdoc
│   │   └── Main_Dishes.gdoc
│   ├── Reference/
│   │   ├── User_Manual.pdf
│   │   └── Quick_Guide.gdoc
│   └── Notes.gdoc
└── Private/
    ├── Medical/
    │   └── Health_Records.pdf
    ├── Financial/
    │   └── Tax_Info.gdoc
    └── Personal_Journal.gdoc
```

All documents and PDFs in subfolders are automatically synced and organized.

### Selective Folder Sync

Disable public or private folder sync by editing `docs_config.json`:

```json
{
  "folders": {
    "public": {
      "enabled": true
    },
    "private": {
      "enabled": false  // Temporarily disabled
    }
  }
}
```

Re-run sync after editing config:
```bash
python3 rosie/src/rosie_docs_sync.py
```

### Stale Document Cleanup

The sync script automatically removes documents that no longer exist in Drive:

1. Detects all `.md` files in `rosie/knowledge_base/google_docs/`
2. Compares with currently synced document IDs
3. Deletes local files for documents no longer in Drive folders
4. Logs removed documents

**Manual cleanup:**
```bash
# View current synced documents
ls rosie/knowledge_base/google_docs/
ls rosie/knowledge_base/private/google_docs/

# Remove all public documents (will re-sync on next run)
rm -rf rosie/knowledge_base/google_docs/*.md

# Remove all private documents
rm -rf rosie/knowledge_base/private/google_docs/*.md

# Re-sync everything
python3 rosie/src/rosie_docs_sync.py
```

### Document Format

Each synced Google Doc is saved as markdown:

```markdown
# Document Title

**Source:** https://docs.google.com/document/d/[document-id]/edit
**Last Synced:** 2025-12-12 14:30:00

[Plain text content of the Google Doc]
```

**Benefits:**
- Readable format for debugging
- Includes source link for verification
- Timestamp shows freshness
- Compatible with ROSIE's RAG system

### Manual Document Management

**View all synced documents:**
```bash
# Public documents
ls rosie/knowledge_base/google_docs/

# Private documents
ls rosie/knowledge_base/private/google_docs/
```

**View specific document:**
```bash
cat rosie/knowledge_base/google_docs/Recipe_Collection.md
cat rosie/knowledge_base/private/google_docs/Medical_Records.md
```

**Check document metadata:**
```bash
head -n 5 rosie/knowledge_base/google_docs/Recipe_Collection.md
```

**Force complete re-sync:**
```bash
# Remove all synced documents (public and private)
rm -rf rosie/knowledge_base/google_docs/*.md
rm -rf rosie/knowledge_base/private/google_docs/*.md

# Re-sync everything
python3 rosie/src/rosie_docs_sync.py

# Restart ROSIE to rebuild RAG index
```

**Add new documents:**
```bash
# Just add documents to Rosie_Knowledge/Public or Rosie_Knowledge/Private in Google Drive
# Then sync to download them
python3 rosie/src/rosie_docs_sync.py
```

## Security & Privacy

### What Gets Committed to Git
**NOTHING docs-related gets committed** (protected by .gitignore):
- ✗ ~/.bashrc (contains all credentials and tokens - GOOGLE_DOCS_TOKEN)
- ✗ rosie/data/docs_config.json (your folder selections)
- ✗ rosie/knowledge_base/google_docs/*.md (your documents)

### What's Safe to Share
✓ Script files (rosie/src/rosie_docs_*.py)
✓ Documentation (this file)
✓ .gitignore configuration

### Credential Security

**OAuth Client Credentials (in ~/.bashrc):**
- `GOOGLE_DOCS_CLIENT_ID` - Identifies your Google Cloud project
- `GOOGLE_DOCS_CLIENT_SECRET` - Secret key for OAuth flow
- These are like API keys - keep them private
- Never commit ~/.bashrc to git
- **Can reuse Calendar credentials** if already set up

**OAuth Token (in ~/.bashrc):**
- `GOOGLE_DOCS_TOKEN` contains refresh token (long-lived access)
- Treat like a password - grants read access to your Drive
- Never share or commit
- To revoke: Remove from ~/.bashrc and revoke in Google account settings
- Re-authenticate to generate new token: run `rosie/src/rosie_docs_setup.py`

**Why environment variables?**
- Matches ROSIE's existing security pattern (see PIPER_MODEL_PATH, etc.)
- Prevents accidentally committing credentials
- Easier to rotate credentials (just update ~/.bashrc)
- No credential files to track in git

**Permissions:**
- Scripts request read-only Drive access
- Cannot modify or delete your documents
- Only reads Google Docs (no other file types)
- Access limited to selected folders

### API Rate Limits
Google Drive API free tier:
- 1,000,000,000 queries per day
- 1,000 queries per 100 seconds per user

With our setup (sync on startup + optional 6-hour cron):
- ~50-200 API calls per sync (depends on document count)
- 4-8 syncs per day = 200-1,600 API calls/day
- Well within free tier limits

### Data Privacy

**Local storage:**
- Documents stored as plain text markdown files
- Only accessible on your local machine
- No cloud storage or external services (except Google Drive sync)

**Public vs Private folders:**
- **Public** (`Rosie_Knowledge/Public` → `knowledge_base/google_docs/`): General information, always accessible to ROSIE
- **Private** (`Rosie_Knowledge/Private` → `knowledge_base/private/google_docs/`): Sensitive information, only accessible when ROSIE is in private mode
- RAG indexing respects privacy: excludes `**/private/*` when not in private mode
- Same privacy model as local knowledge base files

**What ROSIE knows:**
- Content of all synced documents (Google Docs and PDFs)
- Document titles and folder organization
- When documents were last synced
- Private content ONLY when in private mode

**What ROSIE doesn't know:**
- Documents outside `Rosie_Knowledge/Public` and `Rosie_Knowledge/Private`
- Google Sheets, Slides, or other file types
- Document edit history or permissions
- Who shared documents with you
- Private documents when not in private mode

## Comparison with Other Document Access Methods

| Feature | Google Docs Sync | Manual Copy/Paste | Drive API Direct |
|---------|-----------------|-------------------|------------------|
| **Setup complexity** | Medium (OAuth) | Low | High |
| **Access latency** | <100ms (RAG) | <100ms (RAG) | 1-3 seconds (API) |
| **Updates** | Automated | Manual | Real-time |
| **Offline access** | Yes | Yes | No |
| **Shared docs** | Yes | Manual | Yes |
| **Folder organization** | Preserved | Lost | Preserved |

**When to use Google Docs sync:**
- Access multiple documents via voice
- Documents update regularly (daily/weekly)
- Want automated synchronization
- Need shared document access

**When to use manual copy:**
- One-off document questions
- Document never changes
- Don't want Google API setup

## Future Enhancements

Possible additions (not yet implemented):
- [ ] Support for Google Sheets (spreadsheet data)
- [ ] Support for Google Slides (presentation content)
- [ ] Document creation via voice command
- [ ] Smart document suggestions based on conversation
- [ ] Multi-user document sharing in household ROSIE setup
- [ ] Document version tracking and comparison
- [ ] OCR for image-only PDFs (currently unsupported)

## Support

For issues or questions:
1. Check troubleshooting section above
2. Review console output from ROSIE
3. Check cron logs: `/tmp/rosie_docs_sync.log` (if using cron)
4. View synced documents: `ls rosie/knowledge_base/google_docs/`
5. Verify configuration: `cat rosie/data/docs_config.json`

Common log locations:
- ROSIE console output (startup sync messages)
- `/tmp/rosie_docs_sync.log` (cron sync log)
- Document content: `rosie/knowledge_base/google_docs/`
- Configuration: `rosie/data/docs_config.json`

## Integration with Calendar

If you've already set up Google Calendar integration, the systems work together seamlessly:

**Shared credentials:**
```bash
# These work for both Calendar and Docs:
export GOOGLE_CALENDAR_CLIENT_ID='your-client-id'
export GOOGLE_CALENDAR_CLIENT_SECRET='your-client-secret'

# Or use separate credentials:
export GOOGLE_DOCS_CLIENT_ID='different-client-id'
export GOOGLE_DOCS_CLIENT_SECRET='different-client-secret'
```

**Sync order on ROSIE startup:**
1. Google Calendar events sync (if configured)
2. Google Docs sync (if configured)
3. RAG index rebuilds with all content
4. ROSIE ready with both calendars and documents

**Combined usage:**
```
You: "Rosie, what's my schedule today and what does my meeting notes document say?"
ROSIE: "You have Team Standup at 9 AM and Client Review at 2 PM. Your meeting notes document mentions the client review will cover Q4 budget and the new project proposal."
```
