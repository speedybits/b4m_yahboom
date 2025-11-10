#!/usr/bin/env python3
"""
ROSIE Calendar Event Creation Script
Processes queued event creation requests and creates them via Google Calendar API

This script:
1. Monitors calendar_create_queue.json for pending events
2. Parses event details (title, date/time, duration, calendar)
3. Creates events via Google Calendar API
4. Logs success/failures
5. Removes processed events from queue

Run manually or via cron job (recommended: every 5 minutes)
"""

import os
import json
import datetime
import re
from pathlib import Path
from google.auth.transport.requests import Request
from google.oauth2.credentials import Credentials
from googleapiclient.discovery import build
from googleapiclient.errors import HttpError

# File paths
SCRIPT_DIR = Path(__file__).parent
CONFIG_FILE = SCRIPT_DIR / 'calendar_config.json'
QUEUE_FILE = SCRIPT_DIR / 'calendar_create_queue.json'
LOG_FILE = SCRIPT_DIR / 'calendar_create_log.txt'

# Scopes for read/write access
SCOPES = ['https://www.googleapis.com/auth/calendar']


def get_token_data():
    """Get stored token data from environment variable"""
    token_json = os.getenv('GOOGLE_CALENDAR_TOKEN')
    if token_json:
        try:
            return json.loads(token_json)
        except json.JSONDecodeError:
            print("[AUTH] Warning: Invalid token data in GOOGLE_CALENDAR_TOKEN")
            return None
    return None


def save_token_data(creds):
    """Save refreshed token data back to environment (for this session)"""
    token_json = creds.to_json()
    os.environ['GOOGLE_CALENDAR_TOKEN'] = token_json
    print("\n[AUTH] Token was refreshed. Update your ~/.bashrc with:")
    print(f"export GOOGLE_CALENDAR_TOKEN='{token_json}'")


def load_config():
    """Load calendar configuration"""
    if not CONFIG_FILE.exists():
        print(f"[ERROR] Configuration file not found: {CONFIG_FILE}")
        return None

    with open(CONFIG_FILE, 'r') as f:
        return json.load(f)


def load_queue():
    """
    Load event creation queue

    Returns:
        List of event creation requests, or empty list if no queue
    """
    if not QUEUE_FILE.exists():
        return []

    try:
        with open(QUEUE_FILE, 'r') as f:
            content = f.read().strip()
            if not content:
                return []
            queue = json.loads(content)
            return queue if isinstance(queue, list) else []
    except json.JSONDecodeError:
        print(f"[ERROR] Invalid JSON in queue file")
        return []


def save_queue(queue):
    """Save updated queue back to file"""
    with open(QUEUE_FILE, 'w') as f:
        json.dump(queue, f, indent=2)


def log_event(message):
    """Append message to log file"""
    timestamp = datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')
    with open(LOG_FILE, 'a') as f:
        f.write(f"[{timestamp}] {message}\n")


def authenticate():
    """Authenticate with Google Calendar API using environment variable"""
    token_data = get_token_data()
    if not token_data:
        print(f"[ERROR] GOOGLE_CALENDAR_TOKEN not found in environment!")
        print("Please run rosie_calendar_setup.py first to generate token")
        return None

    creds = Credentials.from_authorized_user_info(token_data, SCOPES)

    if creds.expired and creds.refresh_token:
        print("[AUTH] Refreshing expired credentials...")
        creds.refresh(Request())
        save_token_data(creds)

    return creds


def parse_datetime(date_str, time_str=None, base_date=None):
    """
    Parse natural language date/time strings

    Args:
        date_str: Date string (e.g., "tomorrow", "Friday", "2025-01-15")
        time_str: Time string (e.g., "2pm", "14:30", "2:30 PM")
        base_date: Base datetime for relative dates (defaults to now)

    Returns:
        datetime object, or None if parsing fails
    """
    if base_date is None:
        base_date = datetime.datetime.now()

    date_str = date_str.lower().strip()

    # Relative dates
    if date_str == "today":
        target_date = base_date.date()
    elif date_str == "tomorrow":
        target_date = (base_date + datetime.timedelta(days=1)).date()
    elif date_str in ["monday", "tuesday", "wednesday", "thursday", "friday", "saturday", "sunday"]:
        # Find next occurrence of this weekday
        weekdays = ["monday", "tuesday", "wednesday", "thursday", "friday", "saturday", "sunday"]
        target_weekday = weekdays.index(date_str)
        days_ahead = (target_weekday - base_date.weekday()) % 7
        if days_ahead == 0:
            days_ahead = 7  # Next week if it's the same day
        target_date = (base_date + datetime.timedelta(days=days_ahead)).date()
    else:
        # Try parsing as ISO date (YYYY-MM-DD)
        try:
            target_date = datetime.datetime.fromisoformat(date_str).date()
        except ValueError:
            print(f"[ERROR] Could not parse date: {date_str}")
            return None

    # Parse time if provided
    if time_str:
        time_str = time_str.lower().strip()

        # Try various time formats
        time_patterns = [
            (r'(\d{1,2}):(\d{2})\s*(am|pm)?', lambda m: (
                int(m.group(1)) + (12 if m.group(3) == 'pm' and int(m.group(1)) < 12 else 0),
                int(m.group(2))
            )),
            (r'(\d{1,2})\s*(am|pm)', lambda m: (
                int(m.group(1)) + (12 if m.group(2) == 'pm' and int(m.group(1)) < 12 else 0),
                0
            )),
            (r'(\d{1,2}):(\d{2})', lambda m: (int(m.group(1)), int(m.group(2)))),
        ]

        hour, minute = None, None
        for pattern, parser in time_patterns:
            match = re.search(pattern, time_str)
            if match:
                hour, minute = parser(match)
                break

        if hour is None:
            print(f"[ERROR] Could not parse time: {time_str}")
            return None

        # Fix 12 AM/PM edge cases
        if hour == 12 and 'am' in time_str:
            hour = 0
        elif hour == 24:
            hour = 0

        return datetime.datetime.combine(target_date, datetime.time(hour, minute))
    else:
        # No time specified, default to 9 AM
        return datetime.datetime.combine(target_date, datetime.time(9, 0))


def create_event(service, event_request, config):
    """
    Create a calendar event from a request

    Args:
        service: Google Calendar API service object
        event_request: Event request dictionary from queue
        config: Calendar configuration

    Returns:
        (success: bool, message: str)
    """
    try:
        # Extract event details
        summary = event_request.get('summary', 'Untitled Event')
        date_str = event_request.get('date', 'today')
        time_str = event_request.get('time')
        duration_minutes = event_request.get('duration_minutes', config.get('create_settings', {}).get('default_duration_minutes', 60))
        location = event_request.get('location', '')
        description = event_request.get('description', '')
        calendar_id = event_request.get('calendar_id', config.get('create_settings', {}).get('default_calendar_id', 'primary'))

        print(f"[CREATE] Creating event: {summary}")
        print(f"[CREATE]   Date: {date_str}, Time: {time_str}, Duration: {duration_minutes}min")

        # Parse datetime
        start_dt = parse_datetime(date_str, time_str)
        if not start_dt:
            return False, f"Failed to parse date/time: {date_str} {time_str}"

        end_dt = start_dt + datetime.timedelta(minutes=duration_minutes)

        # Build event object
        event = {
            'summary': summary,
            'start': {
                'dateTime': start_dt.isoformat(),
                'timeZone': 'America/New_York',  # TODO: Make configurable
            },
            'end': {
                'dateTime': end_dt.isoformat(),
                'timeZone': 'America/New_York',
            },
        }

        if location:
            event['location'] = location

        if description:
            event['description'] = description

        # Create event via API
        created_event = service.events().insert(
            calendarId=calendar_id,
            body=event
        ).execute()

        event_link = created_event.get('htmlLink', '')
        msg = f"✓ Created: {summary} on {start_dt.strftime('%A, %B %d at %I:%M %p')}"
        print(f"[CREATE] {msg}")
        log_event(msg)

        return True, msg

    except HttpError as error:
        msg = f"✗ API Error: {error}"
        print(f"[CREATE] {msg}")
        log_event(f"Failed to create '{summary}': {error}")
        return False, msg

    except Exception as e:
        msg = f"✗ Error: {e}"
        print(f"[CREATE] {msg}")
        log_event(f"Failed to create '{summary}': {e}")
        return False, msg


def process_queue(service, config):
    """
    Process all pending events in the queue

    Args:
        service: Google Calendar API service object
        config: Calendar configuration

    Returns:
        (processed_count, success_count)
    """
    queue = load_queue()

    if not queue:
        print("[QUEUE] No pending events")
        return 0, 0

    print(f"[QUEUE] Processing {len(queue)} pending event(s)...")

    processed = []
    success_count = 0

    for event_request in queue:
        success, message = create_event(service, event_request, config)
        if success:
            success_count += 1
            # Mark as processed (remove from queue)
        else:
            # Keep failed events in queue for retry
            processed.append(event_request)

    # Save updated queue (only failed events remain)
    save_queue(processed)

    return len(queue), success_count


def main():
    """Main event creation flow"""
    print("=" * 80)
    print("ROSIE CALENDAR EVENT CREATION")
    print("=" * 80)

    # Load configuration
    config = load_config()
    if not config:
        return

    # Check if there's anything to process
    queue = load_queue()
    if not queue:
        # No output if no queue (for cron job silence)
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

    # Process queue
    total, success = process_queue(service, config)

    print("\n" + "=" * 80)
    print(f"PROCESSED: {success}/{total} events created successfully")
    if total - success > 0:
        print(f"FAILED: {total - success} events remain in queue for retry")
    print("=" * 80)


if __name__ == '__main__':
    main()
