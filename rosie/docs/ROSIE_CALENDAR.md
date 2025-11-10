# ROSIE Google Calendar Integration

Voice-controlled calendar access and event creation for ROSIE Conversational AI.

## Overview

This integration connects ROSIE to your Google Calendar, allowing you to:
- **Ask about events**: "Rosie, what's on my calendar today?"
- **Create events**: "Rosie, schedule a dentist appointment Friday at 2pm"
- **Access shared calendars**: Select which calendars to sync (including shared ones)
- **Automatic syncing**: Events refresh every 15 minutes via cron job

## Architecture

### RAG-Based Reading (Fast, No Latency)
- Calendar events synced to `knowledge_base/calendar_events.md`
- ROSIE's existing RAG system indexes events automatically
- Zero latency during conversation (no API calls)
- Events update every 15 minutes

### Queue-Based Writing (5-15 Minute Delay)
- Voice commands → Ollama extracts event details → Writes to queue file
- Background script processes queue every 5 minutes
- Creates events via Google Calendar API
- Next sync (15 min) updates ROSIE's knowledge base

**Why this approach?**
- Maintains ROSIE's fully-local conversational philosophy
- No external API calls during conversation (no latency/failures)
- Graceful error handling (failed events remain in queue)
- Simple debugging (queue file is plain JSON)

## Setup Guide

### Prerequisites

1. **Python libraries:**
   ```bash
   pip install --upgrade google-api-python-client google-auth-httplib2 google-auth-oauthlib
   ```

2. **Google Cloud Project OAuth Credentials:**
   - Go to https://console.cloud.google.com/
   - Create a new project (or use existing)
   - Enable the Google Calendar API
   - Configure OAuth consent screen (internal use, your email)
   - Create OAuth 2.0 Desktop credentials
   - Copy Client ID and Client Secret

3. **Add credentials to ~/.bashrc:**
   ```bash
   # Open ~/.bashrc
   nano ~/.bashrc

   # Add these lines at the bottom (replace with your actual values):
   export GOOGLE_CALENDAR_CLIENT_ID='your-client-id.apps.googleusercontent.com'
   export GOOGLE_CALENDAR_CLIENT_SECRET='your-client-secret'

   # Save and exit, then reload:
   source ~/.bashrc
   ```

   **Security Note:** Credentials are stored as environment variables in ~/.bashrc instead of files.
   This follows security best practices and matches ROSIE's existing configuration pattern.

### Initial Setup (One-Time)

1. **Run setup wizard:**
   ```bash
   cd /home/mike/projects/b4m_yahboom
   python3 rosie_calendar_setup.py
   ```

   This will:
   - Open browser for one-time Google authentication
   - List all available calendars (including shared ones)
   - Let you select which calendars to sync with ROSIE
   - Save configuration to `calendar_config.json`
   - **Generate authentication token and print export command**

2. **Save the generated token to ~/.bashrc:**

   After setup completes, you'll see:
   ```bash
   ================================================================================
   TOKEN GENERATED - SAVE TO ~/.bashrc FOR PERSISTENCE
   ================================================================================

   Add this line to your ~/.bashrc:

   export GOOGLE_CALENDAR_TOKEN='{"token": "ya29...", "refresh_token": "1//0g...", ...}'
   ```

   Copy that export line and add it to your ~/.bashrc:
   ```bash
   nano ~/.bashrc
   # Paste the GOOGLE_CALENDAR_TOKEN line at the bottom
   # Save and exit
   source ~/.bashrc
   ```

3. **Test manual sync:**
   ```bash
   python3 rosie_calendar_sync.py
   ```

   Verify events appear in:
   ```bash
   cat knowledge_base/calendar_events.md
   ```

4. **Set up automatic syncing (recommended):**
   ```bash
   crontab -e
   ```

   Add these lines:
   ```bash
   # Sync calendar events every 15 minutes
   */15 * * * * cd /home/mike/projects/b4m_yahboom && python3 rosie_calendar_sync.py >> /tmp/rosie_calendar_sync.log 2>&1

   # Process event creation queue every 5 minutes
   */5 * * * * cd /home/mike/projects/b4m_yahboom && python3 rosie_calendar_create.py >> /tmp/rosie_calendar_create.log 2>&1
   ```

   Save and exit. Verify cron jobs:
   ```bash
   crontab -l
   ```

## Usage

### Reading Calendar Events

Ask ROSIE about your calendar:

```
You: "Rosie, what's on my calendar today?"
ROSIE: "You have Team Standup at 9 AM and Dentist Appointment at 2 PM."

You: "Rosie, when is my next meeting?"
ROSIE: "Your next meeting is Team Standup tomorrow at 9 AM."

You: "Rosie, do I have anything scheduled Friday?"
ROSIE: "Yes, you have three events on Friday: Project Review at 10 AM, Lunch with Sarah at noon, and Kids Soccer Practice at 6 PM."
```

**How it works:**
- Events are stored in `knowledge_base/calendar_events.md`
- ROSIE's RAG system retrieves relevant events
- No API calls during conversation (instant response)

### Creating Calendar Events

Tell ROSIE to schedule something:

```
You: "Rosie, schedule a dentist appointment Friday at 2pm"
ROSIE: "I've queued your dentist appointment for Friday at 2pm. It will be added to your calendar shortly."

[Background script creates event within 5 minutes]

[15 minutes later, sync updates knowledge base]

You: "Rosie, what's on my calendar Friday?"
ROSIE: "You have dentist appointment at 2 PM." [confirmed from actual calendar]
```

**Supported phrases:**
- "schedule a [event] [when]"
- "add [event] to my calendar [when]"
- "create a meeting [when]"
- "book an appointment [when]"
- "set up a reminder [when]"

**Date formats:**
- "today" / "tomorrow"
- "Monday" / "Friday" (next occurrence)
- "January 15" / "Jan 15"
- "2025-01-15" (ISO format)

**Time formats:**
- "2pm" / "2:30pm"
- "14:00" / "14:30"
- "2:30 PM" / "2:30 pm"

### Configuration

Edit `calendar_config.json` to customize:

```json
{
  "calendars": [
    {
      "name": "Family Calendar",
      "id": "[email protected]",
      "enabled": true
    },
    {
      "name": "Work Meetings",
      "id": "[email protected]",
      "enabled": true
    }
  ],
  "sync_settings": {
    "days_ahead": 30,
    "days_behind": 7,
    "max_results_per_calendar": 100
  },
  "create_settings": {
    "default_calendar_id": "[email protected]",
    "default_duration_minutes": 60
  }
}
```

**Settings:**
- `enabled`: Set to `false` to disable a calendar without removing it
- `days_ahead`: How far into the future to sync (default: 30 days)
- `days_behind`: How far into the past to sync (default: 7 days)
- `default_calendar_id`: Which calendar to add new events to
- `default_duration_minutes`: Default event length if not specified

## File Structure

```
/home/mike/projects/b4m_yahboom/
├── rosie_calendar_setup.py         # Setup wizard
├── rosie_calendar_sync.py          # Sync events to RAG
├── rosie_calendar_create.py        # Create queued events
├── rosie_conversation.py           # ROSIE (modified for calendar)
├── bashrc_calendar_template.txt    # Template for ~/.bashrc variables
├── calendar_config.json            # Calendar selections (NEVER commit)
├── calendar_create_queue.json      # Pending events (NEVER commit)
├── calendar_create_log.txt         # Creation log (NEVER commit)
└── knowledge_base/
    └── calendar_events.md          # Synced events (NEVER commit)

# Environment variables in ~/.bashrc (NEVER commit .bashrc!)
# GOOGLE_CALENDAR_CLIENT_ID
# GOOGLE_CALENDAR_CLIENT_SECRET
# GOOGLE_CALENDAR_TOKEN
```

## Troubleshooting

### Authentication Issues

**Error: "Google Calendar credentials not found in environment variables!"**
- Make sure you added GOOGLE_CALENDAR_CLIENT_ID and GOOGLE_CALENDAR_CLIENT_SECRET to ~/.bashrc
- Run: `source ~/.bashrc` to load them
- Verify: `echo $GOOGLE_CALENDAR_CLIENT_ID`

**Error: "GOOGLE_CALENDAR_TOKEN not found in environment!"**
- Run `rosie_calendar_setup.py` to generate token
- Copy the export command it prints
- Add to ~/.bashrc and run `source ~/.bashrc`

**Token refresh messages**
- If you see "Token was refreshed. Update your ~/.bashrc with:", copy the new export line
- This happens when the access token expires (every ~1 hour)
- The refresh token automatically gets a new access token
- Update ~/.bashrc with the new token to persist it

### Calendar Not Syncing

**Check if cron job is running:**
```bash
# View sync logs
tail -f /tmp/rosie_calendar_sync.log

# Manually run sync to see errors
python3 rosie_calendar_sync.py
```

**Common issues:**
- Cron path issues: Use full paths in crontab
- Token expired: Re-run `rosie_calendar_setup.py`
- Calendar not enabled: Check `calendar_config.json`

### Events Not Being Created

**Check creation queue and logs:**
```bash
# View pending events
cat calendar_create_queue.json

# View creation logs
tail -f /tmp/rosie_calendar_create.log

# Manually process queue
python3 rosie_calendar_create.py
```

**Common issues:**
- Invalid date/time format: Check Ollama extraction in logs
- API permission error: Re-authenticate with full calendar scope
- Calendar ID wrong: Check `calendar_config.json`

### ROSIE Not Detecting Calendar Requests

**Detection patterns:**
- Must say: "schedule", "add", "create", "set up", or "book"
- Must include: "appointment", "meeting", "event", or "reminder"
- Example: "schedule dentist appointment" ✓
- Example: "remind me to call John" ✓ (contains "reminder")
- Example: "call the dentist Friday" ✗ (no trigger words)

**Check Ollama extraction:**
- Look for `[CALENDAR]` messages in ROSIE console output
- Ollama should extract JSON with event details
- If extraction fails, event won't be queued

### Shared Calendars Not Appearing

**Make sure calendar is shared with your Google account:**
1. Go to https://calendar.google.com
2. Check if calendar appears in "Other calendars" list
3. If not, ask calendar owner to share with your email
4. Minimum permission: "See all event details"

**Re-run setup wizard to detect newly shared calendars:**
```bash
python3 rosie_calendar_setup.py
```

## Advanced Topics

### Multiple Calendar Selection

You can have ROSIE ask which calendar to use when creating events:

```json
{
  "create_settings": {
    "allow_calendar_selection": true
  }
}
```

Then specify calendar in voice command:
```
"Rosie, add team meeting to work calendar tomorrow at 10am"
```

### Custom Event Duration

Specify duration in voice command:
```
"Rosie, schedule 30 minute call with John tomorrow at 2pm"
"Rosie, book 2 hour meeting Friday afternoon"
```

### Location and Description

Add location or details:
```
"Rosie, schedule dentist appointment Friday at 2pm at Downtown Dental"
"Rosie, create project meeting Monday at 9am in conference room B"
```

Ollama will extract location from the phrase.

### Timezone Configuration

Edit `rosie_calendar_create.py` to change timezone (line ~126):

```python
'timeZone': 'America/New_York',  # Change this
```

Common timezones:
- `America/New_York` (Eastern)
- `America/Chicago` (Central)
- `America/Denver` (Mountain)
- `America/Los_Angeles` (Pacific)

### Manual Event Management

**View all events:**
```bash
cat knowledge_base/calendar_events.md
```

**View pending events:**
```bash
cat calendar_create_queue.json
```

**Clear failed events from queue:**
```bash
echo "[]" > calendar_create_queue.json
```

**Force immediate sync:**
```bash
python3 rosie_calendar_sync.py
```

**Process creation queue immediately:**
```bash
python3 rosie_calendar_create.py
```

## Security & Privacy

### What Gets Committed to Git
**NOTHING calendar-related gets committed** (protected by .gitignore):
- ✗ ~/.bashrc (contains all credentials and tokens)
- ✗ calendar_config.json (your calendar selections)
- ✗ calendar_create_queue.json (pending events)
- ✗ knowledge_base/calendar_events.md (your events)

### What's Safe to Share
✓ Script files (rosie_calendar_*.py)
✓ Documentation (this file)
✓ .gitignore configuration
✓ bashrc_calendar_template.txt (template with placeholders)

### Credential Security

**OAuth Client Credentials (in ~/.bashrc):**
- `GOOGLE_CALENDAR_CLIENT_ID` - Identifies your Google Cloud project
- `GOOGLE_CALENDAR_CLIENT_SECRET` - Secret key for OAuth flow
- These are like API keys - keep them private
- Never commit ~/.bashrc to git

**OAuth Token (in ~/.bashrc):**
- `GOOGLE_CALENDAR_TOKEN` contains refresh token (long-lived access)
- Treat like a password - grants access to your calendars
- Never share or commit
- To revoke: Remove from ~/.bashrc and revoke in Google account settings
- Re-authenticate to generate new token: run `rosie_calendar_setup.py`

**Why environment variables?**
- Matches ROSIE's existing security pattern (see PIPER_MODEL_PATH, etc.)
- Prevents accidentally committing credentials
- Easier to rotate credentials (just update ~/.bashrc)
- No credential files to track in git

### API Rate Limits
Google Calendar API free tier:
- 1,000,000 queries per day
- 10 queries per second

With our setup (sync every 15 min, create every 5 min):
- ~200 API calls per day
- Well within free tier limits

## Comparison with Direct API Integration

| Feature | Current (Queue-Based) | Direct API |
|---------|----------------------|------------|
| **Read latency** | <100ms (RAG) | <100ms (RAG) |
| **Write latency** | 5-15 minutes | Immediate |
| **Conversation blocking** | No API calls | Blocks for API |
| **Error handling** | Graceful (retry queue) | Must handle in conversation |
| **Complexity** | Medium | High |
| **Debugging** | Easy (queue file) | Hard (logs only) |
| **ROSIE architecture** | Maintains local philosophy | Adds external dependencies |

**When to upgrade to Direct API:**
- Need instant event creation confirmation (<1 second)
- Complex calendar operations (moving events, conflict detection)
- Multi-calendar event creation with user selection

## Future Enhancements

Possible additions (not yet implemented):
- [ ] Event modification/deletion via voice
- [ ] Recurring event support
- [ ] Conflict detection
- [ ] Calendar search by attendee
- [ ] Integration with other calendar services (Outlook, iCal)
- [ ] Smart event suggestions based on conversation history

## Support

For issues or questions:
1. Check troubleshooting section above
2. Review console output from ROSIE
3. Check cron logs: `/tmp/rosie_calendar_*.log`
4. View queue status: `cat calendar_create_queue.json`

Common log locations:
- ROSIE console output
- `/tmp/rosie_calendar_sync.log`
- `/tmp/rosie_calendar_create.log`
- `calendar_create_log.txt` (event creation history)
