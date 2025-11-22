# ROSIE Calendar Integration - Quick Setup

## What You Need to Do

### Step 1: Install Python Libraries
```bash
pip install --upgrade google-api-python-client google-auth-httplib2 google-auth-oauthlib
```

### Step 2: Create Google Cloud Project & Get Credentials

1. **Go to Google Cloud Console:**
   - Visit: https://console.cloud.google.com/

2. **Create or Select Project:**
   - Click "Select a project" → "New Project"
   - Name it (e.g., "ROSIE Calendar")
   - Click "Create"

3. **Enable Google Calendar API:**
   - In the search bar, type "Google Calendar API"
   - Click on it and press "Enable"

4. **Configure OAuth Consent Screen:**
   - Go to "APIs & Services" → "OAuth consent screen"
   - Select "Internal" (if available) or "External"
   - Fill in app name: "ROSIE Calendar"
   - Fill in your email
   - Click "Save and Continue" through all steps

5. **Create OAuth 2.0 Credentials:**
   - Go to "APIs & Services" → "Credentials"
   - Click "Create Credentials" → "OAuth client ID"
   - Application type: **Desktop app**
   - Name: "ROSIE Desktop Client"
   - Click "Create"

6. **Copy Credentials to ~/.bashrc:**
   - Click on your newly created OAuth 2.0 Client ID
   - You'll see **Client ID** and **Client secret** on the right side
   - Copy these values (click the copy icon next to each)

   - Open your ~/.bashrc file:
     ```bash
     nano ~/.bashrc
     ```

   - Scroll to the bottom and add these lines (replace with your actual values):
     ```bash
     # Google Calendar API Credentials
     export GOOGLE_CALENDAR_CLIENT_ID='your-client-id-here.apps.googleusercontent.com'
     export GOOGLE_CALENDAR_CLIENT_SECRET='your-client-secret-here'
     ```

   - Save and exit (Ctrl+X, then Y, then Enter)

   - Load the new variables:
     ```bash
     source ~/.bashrc
     ```

   - Verify:
     ```bash
     echo $GOOGLE_CALENDAR_CLIENT_ID
     ```
     (Should print your client ID)

**Note:** We store credentials as environment variables in ~/.bashrc instead of files for security.
See `bashrc_calendar_template.txt` for a template with detailed comments.

### Step 3: Run Setup Wizard
```bash
cd /home/mike/projects/b4m_yahboom
python3 rosie/src/rosie_calendar_setup.py
```

**What happens:**
1. Opens browser for Google authentication (one-time OAuth flow)
2. Lists all your calendars (including shared ones)
3. You select which ones to sync with ROSIE
4. Saves configuration to `calendar_config.json`
5. **Generates authentication token** and prints export command

**IMPORTANT: After setup completes, you'll see output like:**
```bash
================================================================================
TOKEN GENERATED - SAVE TO ~/.bashrc FOR PERSISTENCE
================================================================================

Add this line to your ~/.bashrc:

export GOOGLE_CALENDAR_TOKEN='{"token": "ya29.a0...", "refresh_token": "1//0g...", ...}'

Then run: source ~/.bashrc

IMPORTANT: Keep this token secret - treat it like a password!
================================================================================
```

**Copy that entire export line and add it to your ~/.bashrc:**
```bash
nano ~/.bashrc
# Paste the export GOOGLE_CALENDAR_TOKEN='...' line at the bottom
# Save and exit (Ctrl+X, Y, Enter)
source ~/.bashrc
```

**Example interaction:**
```
AVAILABLE CALENDARS
================================================================================

1. Mike's Calendar [PRIMARY]
   ID: [email protected]
   Access: owner

2. Family Calendar
   ID: family123@group.calendar.google.com
   Access: writer

3. Work Meetings
   ID: work456@group.calendar.google.com
   Access: reader

4. Holidays
   ID: holidays@group.calendar.google.com
   Access: reader

================================================================================

Which calendars would you like to sync with ROSIE?
Enter calendar numbers separated by commas (e.g., 1,2,3)
Or press Enter to select all calendars

Your selection: 2,3
```

### Step 4: Test Manual Sync
```bash
python3 rosie/src/rosie_calendar_sync.py
```

**Check output:**
```bash
cat rosie/knowledge_base/calendar_events.md
```

You should see your calendar events formatted as markdown!

### Step 5: Set Up Automatic Syncing (Recommended)

Open crontab editor:
```bash
crontab -e
```

Add these two lines at the bottom:
```bash
# Sync calendar events every 15 minutes
*/15 * * * * cd /home/mike/projects/b4m_yahboom && python3 rosie/src/rosie_calendar_sync.py >> /tmp/rosie_calendar_sync.log 2>&1

# Process event creation queue every 5 minutes
*/5 * * * * cd /home/mike/projects/b4m_yahboom && python3 rosie/src/rosie_calendar_create.py >> /tmp/rosie_calendar_create.log 2>&1
```

Save and exit (Ctrl+X, then Y, then Enter in nano).

**Verify cron jobs:**
```bash
crontab -l
```

### Step 6: Try It Out!

Start ROSIE:
```bash
cd /home/mike/projects/b4m_yahboom
./rosie/scripts/run.sh
```

**Test reading calendar:**
```
You: "Rosie, what's on my calendar today?"
```

**Test creating event:**
```
You: "Rosie, schedule a dentist appointment Friday at 2pm"
```

ROSIE should respond:
```
"I've queued your dentist appointment for Friday at 2pm. It will be added to your calendar shortly."
```

Wait 5-15 minutes, then check:
```
You: "Rosie, what's on my calendar Friday?"
```

## That's It!

Your calendar is now integrated with ROSIE.

## Quick Reference

**Environment variables in ~/.bashrc (NEVER commit):**
- `GOOGLE_CALENDAR_CLIENT_ID` - OAuth client ID
- `GOOGLE_CALENDAR_CLIENT_SECRET` - OAuth client secret
- `GOOGLE_CALENDAR_TOKEN` - Your access/refresh token (auto-generated)

**Files created (DO NOT commit to git):**
- `rosie/data/calendar_config.json` - Your calendar selections
- `rosie/data/calendar_create_queue.json` - Pending events
- `rosie/knowledge_base/calendar_events.md` - Synced events

**Scripts you can run:**
- `python3 rosie/src/rosie_calendar_setup.py` - Re-run setup
- `python3 rosie/src/rosie_calendar_sync.py` - Manual sync
- `python3 rosie/src/rosie_calendar_create.py` - Process queue manually

**Check logs:**
```bash
tail -f /tmp/rosie_calendar_sync.log
tail -f /tmp/rosie_calendar_create.log
```

## Troubleshooting

**Browser doesn't open during setup:**
- Check if you're running in SSH without X11 forwarding
- Copy the URL from terminal and open in browser manually

**"Google Calendar credentials not found in environment variables!":**
- Make sure you added GOOGLE_CALENDAR_CLIENT_ID and GOOGLE_CALENDAR_CLIENT_SECRET to ~/.bashrc
- Run: `source ~/.bashrc` to load them
- Verify: `echo $GOOGLE_CALENDAR_CLIENT_ID` (should print your client ID)

**Shared calendar not showing:**
- Go to https://calendar.google.com
- Check if it's in your "Other calendars" list
- If not, ask calendar owner to share it with your email
- Minimum permission: "See all event details"
- Run setup wizard again after it's shared

**Events not syncing:**
- Check if cron job is running: `crontab -l`
- View sync logs: `tail /tmp/rosie_calendar_sync.log`
- Try manual sync: `python3 rosie/src/rosie_calendar_sync.py`

**ROSIE not creating events:**
- Check detection: Look for `[CALENDAR]` in ROSIE console
- Check queue: `cat rosie/data/calendar_create_queue.json`
- Check logs: `tail /tmp/rosie_calendar_create.log`
- Try manual: `python3 rosie/src/rosie_calendar_create.py`

## Full Documentation

See `ROSIE_CALENDAR.md` for complete documentation including:
- Architecture details
- Advanced configuration
- Troubleshooting guide
- Security information
- Future enhancements
