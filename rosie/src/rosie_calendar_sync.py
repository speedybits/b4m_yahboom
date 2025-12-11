#!/usr/bin/env python3
"""
ROSIE Calendar Sync Script
Syncs Google Calendar events to ROSIE's RAG knowledge base

This script:
1. Reads calendar_config.json for selected calendars
2. Authenticates with Google Calendar API
3. Fetches events from all enabled calendars
4. Merges and formats events as markdown
5. Writes to knowledge_base/calendar_events.md
6. ROSIE's RAG system automatically indexes the updates

Run manually or via cron job (recommended: every 15 minutes)
"""

import os
import json
import datetime
from pathlib import Path
from google.auth.transport.requests import Request
from google.oauth2.credentials import Credentials
from googleapiclient.discovery import build
from googleapiclient.errors import HttpError

# File paths (relative to rosie root directory - parent of src/)
ROSIE_DIR = Path(__file__).parent.parent
CONFIG_FILE = ROSIE_DIR / 'data' / 'calendar_config.json'
KNOWLEDGE_BASE_DIR = ROSIE_DIR / 'knowledge_base'
OUTPUT_FILE = KNOWLEDGE_BASE_DIR / 'calendar_events.md'

# Scopes for read/write access
SCOPES = ['https://www.googleapis.com/auth/calendar']


def get_token_data():
    """
    Get stored token data from file or environment variable

    Checks in order (file first since it's more likely to be up-to-date):
    1. rosie/data/calendar_token.json file
    2. GOOGLE_CALENDAR_TOKEN environment variable (fallback)

    Returns:
        Dictionary with token data, or None if not found
    """
    # First check token file (more likely to have fresh token after re-auth)
    token_file = ROSIE_DIR / 'data' / 'calendar_token.json'
    if token_file.exists():
        try:
            token_json = token_file.read_text()
            data = json.loads(token_json)
            print("[AUTH] Using token from file")
            return data
        except (json.JSONDecodeError, IOError) as e:
            print(f"[AUTH] Warning: Could not read token file: {e}")

    # Fall back to environment variable
    token_json = os.getenv('GOOGLE_CALENDAR_TOKEN')
    if token_json:
        try:
            print("[AUTH] Using token from environment variable")
            return json.loads(token_json)
        except json.JSONDecodeError:
            print("[AUTH] Warning: Invalid token data in GOOGLE_CALENDAR_TOKEN")

    return None


def save_token_data(creds):
    """
    Save refreshed token data to file and environment

    Args:
        creds: Credentials object with updated token data
    """
    token_json = creds.to_json()
    os.environ['GOOGLE_CALENDAR_TOKEN'] = token_json

    # Save to file for persistence
    token_file = ROSIE_DIR / 'data' / 'calendar_token.json'
    token_file.parent.mkdir(parents=True, exist_ok=True)
    token_file.write_text(token_json)
    print(f"[AUTH] Token refreshed and saved to {token_file}")


def load_config():
    """
    Load calendar configuration from calendar_config.json

    Returns:
        Dictionary with calendar settings, or None if not found
    """
    if not CONFIG_FILE.exists():
        print(f"[ERROR] Configuration file not found: {CONFIG_FILE}")
        print("Please run rosie_calendar_setup.py first")
        return None

    with open(CONFIG_FILE, 'r') as f:
        config = json.load(f)

    print(f"[CONFIG] Loaded configuration with {len(config.get('calendars', []))} calendar(s)")
    return config


def authenticate():
    """
    Authenticate with Google Calendar API using environment variable token

    Returns:
        Credentials object, or None if authentication fails
    """
    token_data = get_token_data()
    if not token_data:
        print(f"[ERROR] GOOGLE_CALENDAR_TOKEN not found in environment!")
        print("Please run rosie_calendar_setup.py first to generate token")
        return None

    creds = Credentials.from_authorized_user_info(token_data, SCOPES)

    # Refresh if expired
    if creds.expired and creds.refresh_token:
        print("[AUTH] Refreshing expired credentials...")
        creds.refresh(Request())
        save_token_data(creds)

    return creds


def fetch_calendar_events(service, calendar_id, calendar_name, settings):
    """
    Fetch events from a single calendar

    Args:
        service: Google Calendar API service object
        calendar_id: Calendar ID to fetch from
        calendar_name: Human-readable calendar name
        settings: Sync settings from config

    Returns:
        List of event dictionaries
    """
    try:
        # Calculate time range
        now = datetime.datetime.now(tz=datetime.timezone.utc)
        days_ahead = settings.get('days_ahead', 30)
        days_behind = settings.get('days_behind', 7)
        max_results = settings.get('max_results_per_calendar', 100)

        time_min = (now - datetime.timedelta(days=days_behind)).isoformat()
        time_max = (now + datetime.timedelta(days=days_ahead)).isoformat()

        print(f"[SYNC] Fetching events from '{calendar_name}'...")

        # Fetch events
        events_result = service.events().list(
            calendarId=calendar_id,
            timeMin=time_min,
            timeMax=time_max,
            maxResults=max_results,
            singleEvents=True,  # Expand recurring events
            orderBy='startTime'
        ).execute()

        events = events_result.get('items', [])
        print(f"[SYNC] Found {len(events)} event(s) in '{calendar_name}'")

        # Add calendar name to each event for reference
        for event in events:
            event['_calendar_name'] = calendar_name
            event['_calendar_id'] = calendar_id

        return events

    except HttpError as error:
        print(f"[ERROR] Failed to fetch events from '{calendar_name}': {error}")
        return []


def format_datetime(dt_string, is_all_day=False):
    """
    Format datetime string for display

    Args:
        dt_string: ISO format datetime string
        is_all_day: Whether this is an all-day event

    Returns:
        Formatted string
    """
    if is_all_day:
        # Date only (no time)
        dt = datetime.datetime.fromisoformat(dt_string.replace('Z', '+00:00'))
        return dt.strftime('%A, %B %d, %Y')
    else:
        # Date and time
        dt = datetime.datetime.fromisoformat(dt_string.replace('Z', '+00:00'))
        # Convert to local time
        local_dt = dt.astimezone()
        return local_dt.strftime('%A, %B %d, %Y at %I:%M %p')


def format_event_markdown(event):
    """
    Format a single event as markdown

    Args:
        event: Event dictionary from Google Calendar API

    Returns:
        Markdown formatted string
    """
    summary = event.get('summary', 'Untitled Event')
    calendar_name = event.get('_calendar_name', 'Unknown Calendar')

    # Handle start time (dateTime for timed events, date for all-day)
    start = event.get('start', {})
    if 'dateTime' in start:
        start_str = format_datetime(start['dateTime'], is_all_day=False)
        is_all_day = False
    elif 'date' in start:
        start_str = format_datetime(start['date'], is_all_day=True)
        is_all_day = True
    else:
        start_str = 'Unknown time'
        is_all_day = False

    # Handle end time (for duration display)
    end = event.get('end', {})
    if 'dateTime' in end and not is_all_day:
        end_dt = datetime.datetime.fromisoformat(end['dateTime'].replace('Z', '+00:00')).astimezone()
        end_time_str = end_dt.strftime('%I:%M %p')
        start_dt = datetime.datetime.fromisoformat(start['dateTime'].replace('Z', '+00:00')).astimezone()
        duration = end_dt - start_dt
        duration_str = f" to {end_time_str} ({int(duration.total_seconds() / 60)} minutes)"
    else:
        duration_str = ""

    # Location
    location = event.get('location', '')
    location_str = f"\n  - **Location:** {location}" if location else ""

    # Build markdown (excluding description for privacy)
    md = f"- **{summary}**{duration_str} [{calendar_name}]"
    md += f"\n  - **When:** {start_str}"
    md += location_str

    return md


def group_events_by_date(events):
    """
    Group events by date for organized display

    Args:
        events: List of event dictionaries

    Returns:
        Dictionary mapping date strings to event lists
    """
    grouped = {}

    for event in events:
        # Get date (without time) for grouping
        start = event.get('start', {})
        if 'dateTime' in start:
            dt = datetime.datetime.fromisoformat(start['dateTime'].replace('Z', '+00:00'))
        elif 'date' in start:
            dt = datetime.datetime.fromisoformat(start['date'] + 'T00:00:00+00:00')
        else:
            continue

        date_key = dt.date().isoformat()

        if date_key not in grouped:
            grouped[date_key] = []

        grouped[date_key].append(event)

    return grouped


def generate_markdown(all_events):
    """
    Generate complete markdown document from events

    Args:
        all_events: List of all events from all calendars

    Returns:
        Markdown formatted string
    """
    if not all_events:
        return "# Calendar Events\n\nNo upcoming events found.\n"

    # Sort events by start time
    def get_event_start(event):
        start = event.get('start', {})
        if 'dateTime' in start:
            return datetime.datetime.fromisoformat(start['dateTime'].replace('Z', '+00:00'))
        elif 'date' in start:
            return datetime.datetime.fromisoformat(start['date'] + 'T00:00:00+00:00')
        return datetime.datetime.min

    all_events.sort(key=get_event_start)

    # Group by date
    grouped = group_events_by_date(all_events)

    # Generate markdown
    md = "# Calendar Events\n\n"
    md += f"*Last updated: {datetime.datetime.now().strftime('%A, %B %d, %Y at %I:%M %p')}*\n\n"
    md += f"Total events: {len(all_events)}\n\n"
    md += "---\n\n"

    # Add events grouped by date
    for date_key in sorted(grouped.keys()):
        events = grouped[date_key]
        date_obj = datetime.datetime.fromisoformat(date_key + 'T00:00:00')
        date_header = date_obj.strftime('%A, %B %d, %Y')

        md += f"## {date_header}\n\n"

        for event in events:
            md += format_event_markdown(event) + "\n\n"

    return md


def write_markdown(markdown_content):
    """
    Write markdown to knowledge base

    Args:
        markdown_content: Markdown string to write
    """
    # Ensure knowledge base directory exists
    KNOWLEDGE_BASE_DIR.mkdir(exist_ok=True)

    # Write to file
    with open(OUTPUT_FILE, 'w') as f:
        f.write(markdown_content)

    print(f"[OUTPUT] ✓ Written to {OUTPUT_FILE}")


def main():
    """Main sync flow"""
    print("=" * 80)
    print("ROSIE CALENDAR SYNC")
    print("=" * 80)

    # Load configuration
    config = load_config()
    if not config:
        return

    # Authenticate
    creds = authenticate()
    if not creds:
        return

    # Build Calendar service
    try:
        service = build('calendar', 'v3', credentials=creds)
    except Exception as e:
        print(f"[ERROR] Failed to build Calendar service: {e}")
        return

    # Fetch events from all enabled calendars
    all_events = []
    calendars = config.get('calendars', [])
    sync_settings = config.get('sync_settings', {})

    for calendar in calendars:
        if not calendar.get('enabled', True):
            print(f"[SKIP] Calendar '{calendar.get('name')}' is disabled")
            continue

        events = fetch_calendar_events(
            service,
            calendar.get('id'),
            calendar.get('name'),
            sync_settings
        )
        all_events.extend(events)

    print(f"\n[SYNC] Total events fetched: {len(all_events)}")

    # Generate markdown
    print("[FORMAT] Generating markdown...")
    markdown = generate_markdown(all_events)

    # Write to knowledge base
    write_markdown(markdown)

    print("\n" + "=" * 80)
    print("SYNC COMPLETE!")
    print("=" * 80)
    print(f"\nCalendar events are now available in ROSIE's knowledge base.")
    print(f"ROSIE can answer questions like:")
    print(f"  - 'What's on my calendar today?'")
    print(f"  - 'When is my next meeting?'")
    print(f"  - 'Do I have anything scheduled tomorrow?'")
    print("\n" + "=" * 80)


if __name__ == '__main__':
    main()
